// Copyright Johnlyon
//
// BimFormatStrategy — per-format BIM binding strategies (v2.3 architecture).
//
// One strategy class per source format; all format-specific identity /
// metadata handling lives behind IBindingStrategy so the rest of the
// binding pipeline is format-agnostic (BIM_BINDING_ARCHITECTURE.md §4.2).
//
// The strategy axis answers "where does the ID come from" per format:
//   IFC   — IfcGloballyUniqueId is embedded in the assimp node name tail
//   FBX   — user-defined properties on aiNode::mMetaData; no native GUID
//   glTF2 — extras on aiNode::mMetaData
//   OBJ/3DS — name only
//

#pragma once

#include "macro.h"

#include "../MeshProjectionErrorCorrector/TileDataTypes.h"

#include <memory>
#include <string>
#include <vector>

struct aiScene;
struct aiNode;
struct aiMesh;

// ---------------------------------------------------------------------------
// IBindingStrategy — abstraction layer for per-format binding logic
// ---------------------------------------------------------------------------
class TILES_CONVERTER_API IBindingStrategy
{
public:
    // Capability self-description: single source of truth for --bim-formats
    // (prevents documentation/binary drift — BIM doc §3.2.1 matrix).
    struct Capability
    {
        const char* format = "";     // "IFC" / "FBX" / "glTF2" / "OBJ" / "3DS" / "Generic"
        bool        nativeGuid = false;
        const char* idSource = "";   // human-readable objectId resolution path
        bool        sceneMetadata = false;
        const char* caveats = "";
    };

    // Identity resolution result: value + provenance (for reports).
    struct IdResolution
    {
        enum class Source { None, NameTail, MetadataKey, ObjectName };
        std::string objectId;
        Source      source = Source::None;

        const char* SourceName() const
        {
            switch (source)
            {
            case Source::NameTail:     return "name-tail";
            case Source::MetadataKey:  return "metadata";
            case Source::ObjectName:   return "objectName";
            default:                   return "none";
            }
        }
    };

    virtual ~IBindingStrategy() = default;

    // Row key lookup (shared id-key candidate search). Public: the binding
    // pipeline uses it for --bim-id-property overrides.
    // ACCEPTS numeric metadata values (Int32/Int64/Double) and
    // stringifies them — FBX UDP keys such as ElementId are typed ints in
    // aiNode::mMetaData, and sidecar numeric columns go through ParseCell
    // typing; a String-only check silently lost them (review #5).
    static std::string FindKey(const BimPropertyRow& row,
                               const std::vector<std::string>& keys);

    // Single source of truth for default id candidate keys (review #15):
    // metadata keys and sidecar ID columns share this list. Bare "ID" is
    // deliberately excluded (BIM doc §4.5: 防误命中无关属性列).
    static const std::vector<std::string>& DefaultIdKeys();      // metadata candidates
    static const std::vector<std::string>& SidecarIdKeys();      // sidecar column candidates

    // Format-strength normalization of a candidate id value: String passes
    // through; Int32/Int64/Double are stringified (objectId is forced string,
    // BIM doc §4.8(1)). Empty for Bool/Vec3/empty. `acceptNumeric=false`
    // restricts to strings (reserved for callers needing strict identity).
    static std::string IdValueToString(const BimValue& v, bool acceptNumeric = true);

    virtual Capability Describe() const = 0;

    // L2 objectClass guess (e.g. IFC name head "IfcWall"); empty if unknown.
    virtual std::string ObjectClass(const MeshInstance& inst) const;

    // Identity: resolve objectId from scene-internal data only (no sidecar).
    virtual IdResolution ResolveObjectId(const MeshInstance& inst) const = 0;

    // Join keys for sidecar matching, L2 priority order (first match wins).
    virtual std::vector<std::string> JoinKeys(const MeshInstance& inst) const;

    // Scene-internal property row (incl. ancestor-chain inheritance when
    // enabled). Returns nullptr when this instance has no scene metadata.
    virtual std::shared_ptr<const BimPropertyRow> CollectSceneRow(
        const MeshInstance& inst, bool inheritAncestors) const;

protected:
    // Shared implementation helpers -----------------------------------------

    // Generic aiNode::mMetaData row (used by FBX/glTF/Generic strategies).
    std::shared_ptr<const BimPropertyRow> MetadataRow(const MeshInstance& inst,
                                                      bool inheritAncestors) const;
};

// ---------------------------------------------------------------------------
// Per-format strategies (§3.2.1 matrix as code)
// ---------------------------------------------------------------------------

class TILES_CONVERTER_API IfcBindingStrategy : public IBindingStrategy
{
public:
    // "IfcWall_Basic Wall_2a6f..." -> "2a6f..." (last underscore segment)
    Capability Describe() const override;
    IdResolution ResolveObjectId(const MeshInstance& inst) const override;
    std::string ObjectClass(const MeshInstance& inst) const override;  // name head
    std::shared_ptr<const BimPropertyRow> CollectSceneRow(
        const MeshInstance& inst, bool inheritAncestors) const override;
};

class TILES_CONVERTER_API FbxBindingStrategy : public IBindingStrategy
{
public:
    Capability Describe() const override;
    IdResolution ResolveObjectId(const MeshInstance& inst) const override;  // metadata keys
};

class TILES_CONVERTER_API GltfBindingStrategy : public IBindingStrategy
{
public:
    Capability Describe() const override;
    IdResolution ResolveObjectId(const MeshInstance& inst) const override;
};

class TILES_CONVERTER_API ObjBindingStrategy : public IBindingStrategy
{
public:
    Capability Describe() const override;
    IdResolution ResolveObjectId(const MeshInstance& inst) const override;  // objectName
};

class TILES_CONVERTER_API ThreeDsBindingStrategy : public IBindingStrategy
{
public:
    Capability Describe() const override;
    IdResolution ResolveObjectId(const MeshInstance& inst) const override;
};

class TILES_CONVERTER_API GenericBindingStrategy : public IBindingStrategy
{
public:
    Capability Describe() const override;
    IdResolution ResolveObjectId(const MeshInstance& inst) const override;
};

// ---------------------------------------------------------------------------
// BindingStrategyRegistry — format detection + strategy lookup
// ---------------------------------------------------------------------------
class TILES_CONVERTER_API BindingStrategyRegistry
{
public:
    // assimpFormatId: input file name (extension decides) or an importer id
    // (".ifc", "ifc", "IFC", "IfcImporter" all accepted; case-insensitive).
    // Unknown formats fall back to GenericBindingStrategy (with a warning
    // logged by the caller if desired).
    static const IBindingStrategy& For(const std::string& assimpFormatId);

    // All registered strategies (for --bim-formats output; stable order).
    static std::vector<const IBindingStrategy*> All();
};

// ---------------------------------------------------------------------------
// Text-encoding guard (review #17, BIM doc §8 R1)
// ---------------------------------------------------------------------------
// Assimp hands over node names / FBX UDP strings as raw bytes; legacy
// pipelines on Windows carry GBK. Batch Table JSON must be valid UTF-8 or
// the consumer's JSON.parse throws and the whole tile's properties die.
// FixTextEncoding verifies UTF-8; on failure tries the local ANSI codepage
// (GBK on zh-CN Windows) and otherwise replaces invalid sequences with
// U+FFFD. `replaced` (optional) reports whether bytes were changed.
TILES_CONVERTER_API std::string FixTextEncoding(const std::string& in,
                                                bool* replaced = nullptr);
