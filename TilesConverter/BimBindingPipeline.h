// Copyright Johnlyon
//
// BimBindingPipeline — strategy × sidecar composition (v2.3 architecture).
//
// Pipeline per MeshInstance (BIM_BINDING_ARCHITECTURE.md §4.3-②):
//   sceneRow   = strategy->CollectSceneRow(inst, inheritAncestors)
//   sidecarRow = sidecar?->RowFor(strategy->JoinKeys(inst))
//   row        = merge(sceneRow, sidecarRow)  // sidecar keys override
//                + reserved columns objectId/objectName/objectClass
//   result     = { row, objectId, idSource, matchSource }   // transparency
//
// Sidecar CSV rules (CesiumLab-compatible, §3.5.1 mode ③):
//   UTF-8 (BOM tolerated), comma-separated, first column = join key
//   (unique name / GUID). Column header defines property names. Missing
//   values allowed (,,). RFC4180 quoting supported: fields containing
//   commas/quotes/newlines go in double quotes, "" escapes a quote
//   (Excel "CSV UTF-8" exports rely on this — review #13).

#pragma once

#include "macro.h"
#include "BimFormatStrategy.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// SidecarTableSource — external property table (CSV) joined by strategy keys
// ---------------------------------------------------------------------------
class TILES_CONVERTER_API SidecarTableSource
{
public:
    struct Stats
    {
        size_t rowCount = 0;
        size_t duplicateKeys = 0;   // first row wins, later duplicates warned
        size_t matched = 0;
        size_t unmatched = 0;
        size_t utf8Replaced = 0;    // non-UTF-8 sidecar text repaired (§8 R1)
        std::vector<std::string> unmatchedNames;  // first N for warnings
    };

    // Parse a UTF-8 CSV sidecar. Returns false on fatal errors
    // (file missing / unreadable / no header / empty key column).
    bool Load(const std::string& csvPath, std::string& error);

    // Look up by the first matching join key. Returns nullptr when no row matches.
    std::shared_ptr<const BimPropertyRow> RowFor(const std::vector<std::string>& joinKeys);

    const Stats& Stats() const { return m_stats; }
    size_t rowCount() const { return m_rows.size(); }

private:
    // key -> row (first row wins on duplicate keys)
    std::map<std::string, std::shared_ptr<const BimPropertyRow>> m_rows;
    struct Stats m_stats;
};

// ---------------------------------------------------------------------------
// BimBindingPipeline
// ---------------------------------------------------------------------------
class TILES_CONVERTER_API BimBindingPipeline
{
public:
    struct Options
    {
        bool collectSceneMetadata = true;
        bool inheritAncestors = true;
        std::string idPropertyKeys;      // comma-separated override ("GlobalId,ElementId")
    };

    // Transparency record: where did objectId / properties come from.
    struct BindingResult
    {
        std::shared_ptr<const BimPropertyRow> row;  // merged (nullptr = none)
        std::string objectId;
        const char* idSource = "none";   // "name-tail"/"metadata"/"sidecar"/"objectName"/"none"
        const char* matchSource = "none"; // "scene"/"sidecar"/"merged"/"none"
    };

    // Summary printed at the end of BindAll (and into --bim-report).
    struct Summary
    {
        std::string strategyName;
        std::string formatId;
        size_t total = 0;
        size_t withSceneRow = 0;
        size_t withSidecarRow = 0;
        size_t withRow = 0;
        size_t withObjectId = 0;
        // idSource distribution
        std::map<std::string, size_t> idSourceCounts;
        // User keys colliding with the reserved columns; per §4.8(1) the
        // reserved column wins and the collision must be reported.
        size_t reservedCollisions = 0;
        std::vector<std::string> droppedKeys;   // "objectName@Wall001", capped
    };

    BimBindingPipeline(const IBindingStrategy& strategy,
                        SidecarTableSource* sidecar,
                        const Options& options);

    // Binds all instances in place (fills inst.properties). Returns per-instance
    // transparency records aligned with the input vector.
    std::vector<BindingResult> BindAll(std::vector<MeshInstance>& instances,
                                        Summary& summary);

    // --bim-formats output (one line per strategy, stable order).
    static void PrintFormats(std::string& out);

private:
    const IBindingStrategy& m_strategy;
    SidecarTableSource* m_sidecar;
    Options m_options;
};
