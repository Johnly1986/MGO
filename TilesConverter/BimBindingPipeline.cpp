// Copyright Johnlyon
//
// BimBindingPipeline — implementation (see header for the design).

#include "BimBindingPipeline.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace
{
    std::string Trim(const std::string& s)
    {
        size_t a = 0, b = s.size();
        while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
        while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
        return s.substr(a, b - a);
    }

    // Comma-separated token list for --bim-id-property: each entry is
    // trimmed and empties dropped (review #7 — "GlobalId, ElementId" with a
    // space silently never matched before).
    std::vector<std::string> SplitCommaTrim(const std::string& s)
    {
        std::vector<std::string> out;
        std::string cur;
        auto flush = [&]() {
            std::string t = Trim(cur);
            if (!t.empty()) out.push_back(t);
            cur.clear();
        };
        for (char c : s)
        {
            if (c == ',') flush();
            else cur.push_back(c);
        }
        flush();
        return out;
    }

    // RFC4180-style CSV cell: quoted fields may contain commas, doubled
    // quotes and newlines; `quoted` records quoting so ParseCell can skip
    // numeric promotion for explicitly-string cells (review #13).
    struct CsvCell
    {
        std::string text;
        bool quoted = false;
    };

    std::vector<CsvCell> ParseCsvLine(const std::string& line)
    {
        std::vector<CsvCell> out;
        CsvCell cur;
        bool inQuote = false;
        for (size_t i = 0; i < line.size(); ++i)
        {
            char c = line[i];
            if (inQuote)
            {
                if (c == '"')
                {
                    if (i + 1 < line.size() && line[i + 1] == '"') { cur.text.push_back('"'); ++i; }
                    else inQuote = false;
                }
                else cur.text.push_back(c);
            }
            else
            {
                if (c == '"') { inQuote = true; cur.quoted = true; }
                else if (c == ',') { out.push_back(cur); cur = CsvCell{}; }
                else cur.text.push_back(c);
            }
        }
        out.push_back(cur);
        return out;
    }

    bool QuoteBalanceOpen(const std::string& s)
    {
        size_t n = 0;
        for (char c : s) if (c == '"') ++n;
        return (n % 2) != 0;   // "" pairs keep parity; odd => field continues
    }

    void StripUtf8Bom(std::string& s)
    {
        if (s.size() >= 3 &&
            static_cast<unsigned char>(s[0]) == 0xEF &&
            static_cast<unsigned char>(s[1]) == 0xBB &&
            static_cast<unsigned char>(s[2]) == 0xBF)
            s.erase(0, 3);
    }

    // Try to parse a sidecar cell into a typed value. Strings that look like
    // numbers are NOT auto-promoted when quoted (RFC4180 quoting = explicit
    // string intent) and leading-zero tokens stay strings ("0042" is an ID,
    // not 42 — review #5). Only unambiguous bool/int/double tokens are typed.
    BimValue ParseCell(const CsvCell& raw)
    {
        std::string s = Trim(raw.text);
        if (s.empty()) return BimValue::MakeString(std::string());  // missing -> ""
        if (raw.quoted) return BimValue::MakeString(s);             // explicit string

        if (s == "true" || s == "TRUE" || s == "True")  return BimValue::MakeBool(true);
        if (s == "false" || s == "FALSE" || s == "False") return BimValue::MakeBool(false);

        // Leading-zero numeric token: keep as string (ID-style, review #5).
        const bool numericStart = !s.empty() && (std::isdigit(static_cast<unsigned char>(s[0])) || s[0] == '-');
        const bool leadingZero = numericStart &&
            s.find('.') == std::string::npos && s.find('e') == std::string::npos && s.find('E') == std::string::npos &&
            ((s.size() > 1 && s[0] == '0' && std::isdigit(static_cast<unsigned char>(s[1]))) ||
             (s.size() > 2 && s[0] == '-' && s[1] == '0' && std::isdigit(static_cast<unsigned char>(s[2]))));
        if (leadingZero) return BimValue::MakeString(s);

        // Integer? (no leading '+' tolerated to avoid exotic cases)
        if (numericStart)
        {
            errno = 0;
            char* end = nullptr;
            long long v = std::strtoll(s.c_str(), &end, 10);
            if (end && *end == '\0' && errno == 0)
            {
                if (v >= INT32_MIN && v <= INT32_MAX) return BimValue::MakeInt32(static_cast<int32_t>(v));
                return BimValue::MakeInt64(static_cast<int64_t>(v));
            }
        }

        // Double? Must contain '.'/'e' to disambiguate from int above.
        if (s.find('.') != std::string::npos || s.find('e') != std::string::npos ||
            s.find('E') != std::string::npos)
        {
            errno = 0;
            char* end = nullptr;
            double v = std::strtod(s.c_str(), &end);
            if (end && *end == '\0' && errno == 0 && std::isfinite(v))
                return BimValue::MakeDouble(v);
            // "nan"/"inf" tokens: stay strings (review #3 — never seed NaN).
        }

        return BimValue::MakeString(s);
    }
}

// ---------------------------------------------------------------------------
// SidecarTableSource
// ---------------------------------------------------------------------------

bool SidecarTableSource::Load(const std::string& csvPath, std::string& error)
{
    std::ifstream f(csvPath);
    if (!f.is_open())
    {
        error = "cannot open sidecar file: " + csvPath;
        return false;
    }

    auto fixText = [this](std::string s) -> std::string
    {
        bool rep = false;
        std::string out = FixTextEncoding(s, &rep);
        if (rep) ++m_stats.utf8Replaced;
        return out;
    };

    std::string line;
    if (!std::getline(f, line))
    {
        error = "sidecar file is empty: " + csvPath;
        return false;
    }
    StripUtf8Bom(line);   // Excel "CSV UTF-8" files carry a BOM (review #13)

    // Multi-line quoted fields: keep appending while a quote is left open.
    while (QuoteBalanceOpen(line))
    {
        std::string more;
        if (!std::getline(f, more)) break;   // EOF inside quote: salvage
        line += "\n";
        line += more;
    }
    std::vector<CsvCell> headerCells = ParseCsvLine(line);
    if (headerCells.empty() || Trim(headerCells[0].text).empty())
    {
        error = "sidecar CSV has no key column header (first column must be the join key): " + csvPath;
        return false;
    }
    std::vector<std::string> headers;
    headers.reserve(headerCells.size());
    for (auto& h : headerCells) headers.push_back(fixText(Trim(h.text)));

    size_t lineNo = 1;
    while (std::getline(f, line))
    {
        ++lineNo;
        if (lineNo == 2) StripUtf8Bom(line);   // defensive: BOM mid-stream never happens, keep symmetric
        while (QuoteBalanceOpen(line))
        {
            std::string more;
            if (!std::getline(f, more)) break;
            ++lineNo;
            line += "\n";
            line += more;
        }
        if (Trim(line).empty()) continue;

        std::vector<CsvCell> cells = ParseCsvLine(line);
        // Ragged rows: pad with empty cells (row-level tolerance, §4.8(5)).
        while (cells.size() < headers.size()) cells.emplace_back();
        if (cells.size() > headers.size())
            cells.resize(headers.size());   // extra columns dropped (§4.8(5) row tolerance)

        std::string key = fixText(Trim(cells[0].text));
        if (key.empty()) continue;

        auto row = std::make_shared<BimPropertyRow>();
        for (size_t c = 1; c < headers.size(); ++c)
        {
            if (headers[c].empty()) continue;
            BimValue v = ParseCell(cells[c]);
            if (v.type == BimValue::Type::String) v.s = fixText(v.s);
            // Skip fully-empty trailing cells so they don't shadow scene values.
            if (v.type == BimValue::Type::String && v.s.empty()) continue;
            row->emplace_back(headers[c], std::move(v));
        }

        if (m_rows.count(key))
        {
            ++m_stats.duplicateKeys;   // first row wins (R2)
        }
        else
        {
            m_rows.emplace(key, std::move(row));
            ++m_stats.rowCount;
        }
    }

    return true;
}

std::shared_ptr<const BimPropertyRow> SidecarTableSource::RowFor(
    const std::vector<std::string>& joinKeys)
{
    for (const auto& k : joinKeys)
    {
        if (k.empty()) continue;
        auto it = m_rows.find(k);
        if (it != m_rows.end())
        {
            ++m_stats.matched;
            return it->second;
        }
    }
    ++m_stats.unmatched;
    if (m_stats.unmatched <= 20) m_stats.unmatchedNames.push_back(joinKeys.empty() ? "" : joinKeys.front());
    return nullptr;
}

// ---------------------------------------------------------------------------
// BimBindingPipeline
// ---------------------------------------------------------------------------

BimBindingPipeline::BimBindingPipeline(const IBindingStrategy& strategy,
                                        SidecarTableSource* sidecar,
                                        const Options& options)
    : m_strategy(strategy), m_sidecar(sidecar), m_options(options)
{
}

std::vector<BimBindingPipeline::BindingResult>
BimBindingPipeline::BindAll(std::vector<MeshInstance>& instances, Summary& summary)
{
    std::vector<BindingResult> results;
    results.reserve(instances.size());

    // --bim-id-property candidate keys (trimmed; review #7)
    std::vector<std::string> idKeyOverride;
    if (!m_options.idPropertyKeys.empty())
        idKeyOverride = SplitCommaTrim(m_options.idPropertyKeys);

    for (auto& inst : instances)
    {
        BindingResult r;

        // 1. Scene row (format-specific collection + inheritance)
        std::shared_ptr<const BimPropertyRow> sceneRow;
        if (m_options.collectSceneMetadata)
            sceneRow = m_strategy.CollectSceneRow(inst, m_options.inheritAncestors);
        if (sceneRow) ++summary.withSceneRow;

        // 2. Sidecar row (joined by strategy-provided L2 keys)
        std::shared_ptr<const BimPropertyRow> sidecarRow;
        if (m_sidecar)
            sidecarRow = m_sidecar->RowFor(m_strategy.JoinKeys(inst));
        if (sidecarRow) ++summary.withSidecarRow;

        // 3. objectId resolution — the documented priority of §4.8(1)
        //    (review #6 fixed the old inversion where the strategies'
        //    objectName fallback ran BEFORE the sidecar ID column was tried,
        //    so priority ③ could never win):
        //      ① name-tail GlobalId (IFC)
        //      ② explicit --bim-id-property keys, searched in scene metadata
        //        AND sidecar columns (doc §4.5; review #7)
        //      ② strategy-default metadata keys (FBX UDP / glTF extras / IFC Psets)
        //      ③ sidecar ID column (GlobalId/ObjectId/ElementId/ifcGUID/UniqueId)
        //      ④ weak fallback: objectName (only when nothing above matched)
        const IBindingStrategy::IdResolution sid = m_strategy.ResolveObjectId(inst);
        using Src = IBindingStrategy::IdResolution::Source;
        const bool sceneHasId = !sid.objectId.empty();
        bool haveId = false;

        if (sceneHasId && sid.source == Src::NameTail)          // ①
        {
            r.objectId = sid.objectId;
            r.idSource = sid.SourceName();
            haveId = true;
        }
        if (!haveId && !idKeyOverride.empty())                  // ② user-explicit
        {
            if (sceneRow)
            {
                std::string v = IBindingStrategy::FindKey(*sceneRow, idKeyOverride);
                if (!v.empty()) { r.objectId = v; r.idSource = "metadata"; haveId = true; }
            }
            if (!haveId && sidecarRow)
            {
                std::string v = IBindingStrategy::FindKey(*sidecarRow, idKeyOverride);
                if (!v.empty()) { r.objectId = v; r.idSource = "sidecar"; haveId = true; }
            }
        }
        if (!haveId && sceneHasId && sid.source == Src::MetadataKey)   // ② strategy default
        {
            r.objectId = sid.objectId;
            r.idSource = sid.SourceName();
            haveId = true;
        }
        if (!haveId && sceneRow)   // ② default keys over the full inherited scene row
        {
            std::string v = IBindingStrategy::FindKey(*sceneRow, IBindingStrategy::DefaultIdKeys());
            if (!v.empty()) { r.objectId = v; r.idSource = "metadata"; haveId = true; }
        }
        if (!haveId && sidecarRow)                                 // ③
        {
            std::string v = IBindingStrategy::FindKey(*sidecarRow, IBindingStrategy::SidecarIdKeys());
            if (!v.empty()) { r.objectId = v; r.idSource = "sidecar"; haveId = true; }
        }
        if (!haveId && sceneHasId)                                 // ④
        {
            r.objectId = sid.objectId;
            r.idSource = sid.SourceName();
            haveId = true;
        }
        if (haveId) ++summary.withObjectId;
        ++summary.idSourceCounts[r.idSource];

        // 4. Merge rows: scene -> sidecar (sidecar overrides same keys)
        std::shared_ptr<const BimPropertyRow> merged;
        if (sceneRow && sidecarRow)
        {
            auto combined = std::make_shared<BimPropertyRow>(*sceneRow);
            for (const auto& kv : *sidecarRow)
            {
                bool replaced = false;
                for (auto& out : *combined)
                {
                    if (out.first == kv.first) { out.second = kv.second; replaced = true; break; }
                }
                if (!replaced) combined->push_back(kv);
            }
            merged = combined;
            r.matchSource = "merged";
        }
        else if (sidecarRow)
        {
            merged = sidecarRow;
            r.matchSource = "sidecar";
        }
        else if (sceneRow)
        {
            merged = sceneRow;
            r.matchSource = "scene";
        }

        // 5. Reserved-column injection (objectId forced string).
        //    User keys colliding with reserved names are DROPPED here — the
        //    reserved column must win and the collision be reported (review
        //    #8, BIM doc §4.8(1)). Previously the injected objectName landed
        //    after the user copy and first-occurrence readers picked the
        //    wrong one; rows also grew duplicate keys that corrupted --bim-report.
        if (merged)
        {
            static const char* kReserved[] = { "objectId", "objectName", "objectClass" };
            auto withReserved = std::make_shared<BimPropertyRow>();
            withReserved->reserve(merged->size() + 3);
            for (const auto& kv : *merged)
            {
                bool collides = false;
                for (const char* rn : kReserved)
                    if (kv.first == rn) { collides = true; break; }
                if (collides)
                {
                    ++summary.reservedCollisions;
                    if (summary.droppedKeys.size() < 20)
                        summary.droppedKeys.push_back(kv.first + "@" +
                                                      (inst.objectName.empty() ? inst.meshName : inst.objectName));
                    continue;
                }
                withReserved->push_back(kv);
            }
            // Reserved head: stable column order (system layer first).
            withReserved->emplace(withReserved->begin(), "objectId", BimValue::MakeString(r.objectId));
            withReserved->emplace_back("objectName", BimValue::MakeString(inst.objectName));
            std::string objClass = m_strategy.ObjectClass(inst);
            if (!objClass.empty())
                withReserved->emplace_back("objectClass", BimValue::MakeString(objClass));
            merged = withReserved;
            ++summary.withRow;
        }

        r.row = merged;
        inst.properties = merged;
        results.push_back(std::move(r));
    }

    summary.total = instances.size();
    return results;
}

void BimBindingPipeline::PrintFormats(std::string& out)
{
    out.clear();
    for (const auto* s : BindingStrategyRegistry::All())
    {
        auto c = s->Describe();
        char buf[512];
        std::snprintf(buf, sizeof(buf), "%-8s guid=%-3s id=%-70s meta=%-3s %s\n",
                      c.format, c.nativeGuid ? "yes" : "no", c.idSource,
                      c.sceneMetadata ? "yes" : "no", c.caveats);
        out += buf;
    }
}
