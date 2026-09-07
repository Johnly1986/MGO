// Copyright Johnlyon
//
// TileDataTypes — out-of-line implementation of the BIM feature-batch
// helpers declared in TileDataTypes.h (see BIM_BINDING_ARCHITECTURE.md §4.3-③).

#include "TileDataTypes.h"

#include <functional>

namespace
{
    // Hash a full property row (key + value bytes) for content deduplication.
    // Identity columns participate (D2: dedup key = whole row), so object-granular
    // rows collapse only when truly identical.
    void HashCombine(size_t& seed, size_t v)
    {
        seed ^= v + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    }

    size_t RowHash(const BimPropertyRow& row)
    {
        size_t seed = 0x9e3779b97f4a7c15ULL;
        for (const auto& kv : row)
        {
            HashCombine(seed, std::hash<std::string>()(kv.first));
            const BimValue& v = kv.second;
            switch (v.type)
            {
            case BimValue::Type::Bool:   HashCombine(seed, v.b ? 1u : 0u); break;
            case BimValue::Type::Int32:  HashCombine(seed, static_cast<size_t>(static_cast<int64_t>(v.i32))); break;
            case BimValue::Type::Int64:  HashCombine(seed, static_cast<size_t>(v.i64)); break;
            case BimValue::Type::Double: HashCombine(seed, std::hash<double>()(v.d)); break;
            case BimValue::Type::Vec3:
                HashCombine(seed, std::hash<double>()(v.v3[0]));
                HashCombine(seed, std::hash<double>()(v.v3[1]));
                HashCombine(seed, std::hash<double>()(v.v3[2]));
                break;
            case BimValue::Type::String: HashCombine(seed, std::hash<std::string>()(v.s)); break;
            }
        }
        return seed;
    }

    bool RowEqual(const BimPropertyRow& a, const BimPropertyRow& b)
    {
        if (a.size() != b.size()) return false;
        for (size_t i = 0; i < a.size(); ++i)
        {
            if (a[i].first != b[i].first) return false;
            const BimValue& x = a[i].second;
            const BimValue& y = b[i].second;
            if (x.type != y.type) return false;
            switch (x.type)
            {
            case BimValue::Type::Bool:   if (x.b != y.b) return false; break;
            case BimValue::Type::Int32:  if (x.i32 != y.i32) return false; break;
            case BimValue::Type::Int64:  if (x.i64 != y.i64) return false; break;
            case BimValue::Type::Double: if (x.d != y.d) return false; break;
            case BimValue::Type::Vec3:
                if (x.v3[0] != y.v3[0] || x.v3[1] != y.v3[1] || x.v3[2] != y.v3[2]) return false;
                break;
            case BimValue::Type::String: if (x.s != y.s) return false; break;
            }
        }
        return true;
    }
}

uint32_t FeatureBatchTable::AssignBatchId(const std::shared_ptr<const BimPropertyRow>& row)
{
    // Shared-empty-row path: unmatched instances all map to one row (D6).
    if (!row)
    {
        if (!emptyRow)
            emptyRow = std::make_shared<BimPropertyRow>();  // lazily created, NOT guaranteed row 0
        for (size_t i = 0; i < rows.size(); ++i)
            if (rows[i] == emptyRow) return static_cast<uint32_t>(i);
        rows.push_back(emptyRow);
        return static_cast<uint32_t>(rows.size() - 1);
    }

    size_t h = RowHash(*row);
    auto& bucket = m_hashIndex[h];
    for (uint32_t idx : bucket)
    {
        const auto& r = rows[idx];
        // Hash bucket narrows candidates; equality is authoritative.
        if (r == row || (r && RowEqual(*r, *row))) return idx;
    }
    bucket.push_back(static_cast<uint32_t>(rows.size()));
    rows.push_back(row);
    return static_cast<uint32_t>(rows.size() - 1);
}
