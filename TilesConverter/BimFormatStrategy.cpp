// Copyright Johnlyon
//
// BimFormatStrategy — per-format BIM binding strategies (implementation).
//
// Identity rules verified against assimp v6.0.5 sources (see
// BIM_BINDING_ARCHITECTURE.md §3.2.1 / §10 rows 17-20):
//   - IFC node names are "IfcClass_Name_GlobalId" (IFCImporter).
//   - FBX UserProperties / unparsed props land on aiNode::mMetaData typed
//     (FBXConverter::SetupNodeMetadata); internal FBX UniqueId is NOT exposed.
//   - glTF2 extras land on aiNode::mMetaData (glTF2Importer::ParseExtras).
//   - 3DS importer writes no metadata at all; node names can be "$$$DUMMY".

#include "BimFormatStrategy.h"

#include <assimp/scene.h>
#include <assimp/metadata.h>

#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>

#ifdef _WIN32
#include <windows.h>
#endif

#ifdef MGO_HAVE_ICONV
#include <iconv.h>
#include <vector>
#endif

namespace
{
    // -----------------------------------------------------------------------
    // UTF-8 validation / repair (review #17, BIM doc §8 R1).
    // -----------------------------------------------------------------------
    // Length of the valid UTF-8 sequence starting at i, or 0 if invalid.
    size_t Utf8SeqLen(const std::string& s, size_t i)
    {
        unsigned char c = static_cast<unsigned char>(s[i]);
        size_t need; unsigned int cp;
        if (c < 0x80) return 1;
        if ((c & 0xE0) == 0xC0) { need = 1; cp = c & 0x1F; }
        else if ((c & 0xF0) == 0xE0) { need = 2; cp = c & 0x0F; }
        else if ((c & 0xF8) == 0xF0) { need = 3; cp = c & 0x07; }
        else return 0;
        if (i + need >= s.size()) return 0;
        for (size_t k = 1; k <= need; ++k)
        {
            unsigned char cc = static_cast<unsigned char>(s[i + k]);
            if ((cc & 0xC0) != 0x80) return 0;
            cp = (cp << 6) | (cc & 0x3F);
        }
        if (need == 1 && cp < 0x80) return 0;        // overlong
        if (need == 2 && cp < 0x800) return 0;
        if (need == 3 && cp < 0x10000) return 0;
        if (cp > 0x10FFFF) return 0;
        if (cp >= 0xD800 && cp <= 0xDFFF) return 0;  // surrogates
        return need + 1;
    }

    bool IsValidUtf8(const std::string& s)
    {
        for (size_t i = 0; i < s.size(); )
        {
            size_t len = Utf8SeqLen(s, i);
            if (!len) return false;
            i += len;
        }
        return true;
    }

    std::string ReplaceInvalidWithFFFD(const std::string& s)
    {
        std::string out;
        out.reserve(s.size());
        for (size_t i = 0; i < s.size(); )
        {
            size_t len = Utf8SeqLen(s, i);
            if (len) { out.append(s, i, len); i += len; }
            else { out += "\xEF\xBF\xBD"; ++i; }     // U+FFFD
        }
        return out;
    }

#ifdef MGO_HAVE_ICONV
    // POSIX second tier: strict GB18030 (superset of GBK) -> UTF-8. Real
    // assets bake GBK node names (Data/roadbed/root.fbx: ~64k of them);
    // without this they would degrade to U+FFFD on non-Windows machines.
    // Returns "" unless the WHOLE buffer converts without EILSEQ.
    std::string TryGbkToUtf8(const std::string& in)
    {
        iconv_t cd = iconv_open("UTF-8", "GB18030");
        if (cd == (iconv_t)-1) return std::string();
        std::vector<char> out(in.size() * 4 + 8);
        char* inbuf = const_cast<char*>(in.data());
        size_t inleft = in.size();
        char* outbuf = out.data();
        size_t outleft = out.size();
        size_t rc = iconv(cd, &inbuf, &inleft, &outbuf, &outleft);
        iconv_close(cd);
        if (rc == (size_t)-1 || inleft != 0) return std::string();
        return std::string(out.data(), out.size() - outleft);
    }
#endif
}

// Public helper (declared in BimFormatStrategy.h).
std::string FixTextEncoding(const std::string& in, bool* replaced)
{
    if (replaced) *replaced = false;
    if (in.empty() || IsValidUtf8(in)) return in;

#ifdef _WIN32
    // Best effort: legacy exporters bake GBK (the ANSI codepage on zh-CN
    // Windows) into FBX UDP strings. Convert before losing information.
    int wlen = MultiByteToWideChar(CP_ACP, 0, in.data(), static_cast<int>(in.size()), nullptr, 0);
    if (wlen > 0)
    {
        std::wstring w(static_cast<size_t>(wlen), L'\0');
        MultiByteToWideChar(CP_ACP, 0, in.data(), static_cast<int>(in.size()), &w[0], wlen);
        int ulen = WideCharToMultiByte(CP_UTF8, 0, w.data(), wlen, nullptr, 0, nullptr, nullptr);
        if (ulen > 0)
        {
            std::string u(static_cast<size_t>(ulen), '\0');
            WideCharToMultiByte(CP_UTF8, 0, w.data(), wlen, &u[0], ulen, nullptr, nullptr);
            if (IsValidUtf8(u))
            {
                if (replaced) *replaced = true;
                return u;
            }
        }
    }
#endif
#ifdef MGO_HAVE_ICONV
    // POSIX: try GBK/GB18030 before degrading to U+FFFD (see TryGbkToUtf8).
    {
        std::string gbk = TryGbkToUtf8(in);
        if (!gbk.empty() && IsValidUtf8(gbk))
        {
            if (replaced) *replaced = true;
            return gbk;
        }
    }
#endif
    if (replaced) *replaced = true;
    return ReplaceInvalidWithFFFD(in);
}

namespace
{
    // Strip "$AssimpFbx$" and everything after it (pivot-chain suffix),
    // e.g. "Wall$AssimpFbx$_PreRotation" -> "Wall". Caller has already
    // applied this when filling MeshInstance::objectName; kept here for tests.
    std::string StripAssimpFbxTag(const std::string& name)
    {
        size_t pos = name.find("$AssimpFbx$");
        return (pos == std::string::npos) ? name : name.substr(0, pos);
    }

    // Find a metadata row value by candidate keys (first hit wins). Numeric
    // values are stringified via IdValueToString (review #5: FBX UDP keys
    // such as ElementId are typed ints in aiNode::mMetaData, and a
    // String-only check silently lost them). Flattened nested dictionaries
    // are also reachable by leaf name: key "Pset.ifcGUID" matches the
    // candidate "ifcGUID" (suffix form), keeping §4.8(1) lookups working
    // after the review-#16 flattening.
    std::string LookupKeys(const BimPropertyRow& row,
                           const std::vector<std::string>& keys)
    {
        auto keyMatches = [](const std::string& have, const std::string& want)
        {
            if (have == want) return true;
            return have.size() > want.size() + 1 &&
                   have.compare(have.size() - want.size(), want.size(), want) == 0 &&
                   have[have.size() - want.size() - 1] == '.';
        };
        for (const auto& k : keys)
            for (const auto& kv : row)
                if (keyMatches(kv.first, k))
                {
                    std::string v = IBindingStrategy::IdValueToString(kv.second);
                    if (!v.empty()) return v;
                }
        return std::string();
    }

    const aiNode* NodeOf(const MeshInstance& inst)
    {
        return static_cast<const aiNode*>(inst.identityNode);
    }

    // Convert one aiMetadataEntry into a BimValue. AI_AIMETADATA (nested
    // dictionaries) is handled by AppendNodeMetadata's flattening pass and
    // falls through to an empty string here only as a last resort.
    BimValue MetadataToBimValue(const aiMetadataEntry& e)
    {
        switch (e.mType)
        {
        case AI_BOOL:    { bool v;  ::memcpy(&v, e.mData, sizeof(bool));    return BimValue::MakeBool(v); }
        case AI_INT32:  { int32_t v; ::memcpy(&v, e.mData, sizeof(int32_t)); return BimValue::MakeInt32(v); }
        case AI_UINT64:
        {
            uint64_t v; ::memcpy(&v, e.mData, sizeof(uint64_t));
            if (v <= static_cast<uint64_t>(INT64_MAX)) return BimValue::MakeInt64(static_cast<int64_t>(v));
            return BimValue::MakeString(std::to_string(v));   // overflow -> exact digits (review #16)
        }
        case AI_FLOAT:  { float v; ::memcpy(&v, e.mData, sizeof(float));  return BimValue::MakeDouble(static_cast<double>(v)); }
        case AI_DOUBLE: { double v; ::memcpy(&v, e.mData, sizeof(double)); return BimValue::MakeDouble(v); }
        case AI_AISTRING:
        {
            aiString v;
            ::memcpy(&v, e.mData, sizeof(aiString));
            return BimValue::MakeString(FixTextEncoding(std::string(v.C_Str())));
        }
        case AI_AIVECTOR3D:
        {
            aiVector3D v;
            ::memcpy(&v, e.mData, sizeof(aiVector3D));
            return BimValue::MakeVec3(v.x, v.y, v.z);
        }
        case AI_INT64:   { int64_t v; ::memcpy(&v, e.mData, sizeof(int64_t)); return BimValue::MakeInt64(v); }
        case AI_UINT32:  { uint32_t v; ::memcpy(&v, e.mData, sizeof(uint32_t)); return BimValue::MakeInt64(static_cast<int64_t>(v)); }  // widen, no wrap (review #16)
        default:         return BimValue::MakeString(std::string());
        }
    }

    // Append one metadata dictionary flattened with a key prefix.
    // AI_AIMETADATA entries are expanded as "Parent.Child" keys instead of
    // being dropped (BIM doc §4.2; review #16). Depth-capped.
    void AppendMetadataDict(BimPropertyRow& row, const aiMetadata* md,
                            const std::string& prefix, int depth);

    void AppendNodeMetadata(BimPropertyRow& row, const aiNode* node)
    {
        if (!node || !node->mMetaData) return;
        AppendMetadataDict(row, node->mMetaData, std::string(), 0);
    }

    void AppendMetadataDict(BimPropertyRow& row, const aiMetadata* md,
                            const std::string& prefix, int depth)
    {
        if (!md) return;
        for (unsigned int i = 0; i < md->mNumProperties; ++i)
        {
            std::string key(md->mKeys[i].C_Str());
            if (key.empty()) continue;
            if (!prefix.empty()) key = prefix + "." + key;

            const aiMetadataEntry& e = md->mValues[i];

            // Nested dictionary -> recurse with "Parent." prefix (cap depth 3).
            if (e.mType == AI_AIMETADATA && e.mData && depth < 3)
            {
                AppendMetadataDict(row, static_cast<const aiMetadata*>(e.mData), key, depth + 1);
                continue;
            }

            BimValue value = MetadataToBimValue(e);
            // Child overrides same-key parent value; keep original position for
            // byte-stable output (deterministic column order).
            bool replaced = false;
            for (auto& kv : row)
            {
                if (kv.first == key) { kv.second = value; replaced = true; break; }
            }
            if (!replaced) row.emplace_back(key, std::move(value));
        }
    }
}

// ---------------------------------------------------------------------------
// IBindingStrategy — shared defaults
// ---------------------------------------------------------------------------

std::string IBindingStrategy::ObjectClass(const MeshInstance& inst) const
{
    (void)inst;
    return std::string();
}

std::vector<std::string> IBindingStrategy::JoinKeys(const MeshInstance& inst) const
{
    // Default L2 priority: objectId (if resolved) -> objectName -> meshName
    std::vector<std::string> keys;
    IdResolution id = ResolveObjectId(inst);
    if (!id.objectId.empty()) keys.push_back(id.objectId);
    if (!inst.objectName.empty()) keys.push_back(inst.objectName);
    if (!inst.meshName.empty()) keys.push_back(inst.meshName);
    return keys;
}

std::shared_ptr<const BimPropertyRow> IBindingStrategy::CollectSceneRow(
    const MeshInstance& inst, bool inheritAncestors) const
{
    return MetadataRow(inst, inheritAncestors);
}

std::shared_ptr<const BimPropertyRow> IBindingStrategy::MetadataRow(
    const MeshInstance& inst, bool inheritAncestors) const
{
    const aiNode* node = NodeOf(inst);
    if (!node) return nullptr;

    // Quick check: does this node or any ancestor carry metadata?
    auto hasMd = [](const aiNode* n) { return n && n->mMetaData && n->mMetaData->mNumProperties > 0; };

    bool any = hasMd(node);
    if (!any && inheritAncestors)
    {
        for (const aiNode* p = node->mParent; p; p = p->mParent)
            if (hasMd(p)) { any = true; break; }
    }
    if (!any) return nullptr;

    auto row = std::make_shared<BimPropertyRow>();

    // Inheritance: ancestors first (root-most), child keys override.
    if (inheritAncestors)
    {
        std::vector<const aiNode*> chain;
        for (const aiNode* p = node->mParent; p; p = p->mParent)
            chain.push_back(p);
        for (auto it = chain.rbegin(); it != chain.rend(); ++it)
            AppendNodeMetadata(*row, *it);
    }

    AppendNodeMetadata(*row, node);
    return row;
}

std::string IBindingStrategy::FindKey(const BimPropertyRow& row,
                                      const std::vector<std::string>& keys)
{
    return LookupKeys(row, keys);
}

const std::vector<std::string>& IBindingStrategy::DefaultIdKeys()
{
    // Single source of truth (review #15). Bare "ID" excluded per §4.5.
    static const std::vector<std::string> kKeys = { "GlobalId", "ElementId", "ifcGUID", "UniqueId" };
    return kKeys;
}

const std::vector<std::string>& IBindingStrategy::SidecarIdKeys()
{
    // Sidecar ID column candidates (BIM doc §3.5.1 mode ③ priority ③).
    static const std::vector<std::string> kKeys = { "GlobalId", "ObjectId", "ElementId", "ifcGUID", "UniqueId" };
    return kKeys;
}

std::string IBindingStrategy::IdValueToString(const BimValue& v, bool acceptNumeric)
{
    switch (v.type)
    {
    case BimValue::Type::String:
        return v.s;
    case BimValue::Type::Int32:
        return acceptNumeric ? std::to_string(v.i32) : std::string();
    case BimValue::Type::Int64:
        return acceptNumeric ? std::to_string(v.i64) : std::string();
    case BimValue::Type::Double:
        if (!acceptNumeric) return std::string();
        {
            double d = v.d;
            if (std::isfinite(d) && d == std::floor(d) &&
                std::fabs(d) < 9007199254740992.0)   // 2^53: exact in double
                return std::to_string(static_cast<long long>(d));
            char buf[40];
            std::snprintf(buf, sizeof(buf), "%.17g", d);
            return buf;
        }
    default:
        return std::string();   // Bool / Vec3 never serve as IDs
    }
}

// ---------------------------------------------------------------------------
// IfcBindingStrategy — "IfcClass_Name_GlobalId" node names; Psets on metadata
// ---------------------------------------------------------------------------

IBindingStrategy::Capability IfcBindingStrategy::Describe() const
{
    Capability c;
    c.format = "IFC";
    c.nativeGuid = true;
    c.idSource = "node name tail (IfcGloballyUniqueId); Psets via aiNode::mMetaData";
    c.sceneMetadata = true;
    c.caveats = "assimp IFC merges Psets without set-name prefix and stringifies "
                "numeric values — use an explicit schema (--bim-schema) to recover types";
    return c;
}

IBindingStrategy::IdResolution IfcBindingStrategy::ResolveObjectId(const MeshInstance& inst) const
{
    // Name shape: "IfcWall_Basic Wall_0a3tXq2nTCwQxVEQOVJqDz".
    // GlobalId is the last '_'-separated segment. Strictly IFC GUIDs are
    // base64 [0-9A-Za-z_$] of length 22 — validate loosely (>= 8 chars,
    // alphanumeric) to avoid picking up ordinary trailing numbers.
    IdResolution r;
    const std::string& name = inst.objectName.empty() ? inst.meshName : inst.objectName;
    size_t pos = name.rfind('_');
    if (pos == std::string::npos || pos + 1 >= name.size()) return r;

    std::string guid = name.substr(pos + 1);
    if (guid.size() < 8 || guid.size() > 32) return r;
    for (char ch : guid)
    {
        if (!(std::isalnum(static_cast<unsigned char>(ch)) || ch == '_' || ch == '$'))
            return r;
    }

    r.objectId = guid;
    r.source = IdResolution::Source::NameTail;
    return r;
}

std::string IfcBindingStrategy::ObjectClass(const MeshInstance& inst) const
{
    const std::string& name = inst.objectName.empty() ? inst.meshName : inst.objectName;
    if (name.size() < 4 || name.compare(0, 3, "Ifc") != 0) return std::string();
    size_t pos = name.find('_');
    return (pos == std::string::npos || pos <= 3) ? std::string() : name.substr(0, pos);
}

std::shared_ptr<const BimPropertyRow> IfcBindingStrategy::CollectSceneRow(
    const MeshInstance& inst, bool inheritAncestors) const
{
    return MetadataRow(inst, inheritAncestors);
}

// ---------------------------------------------------------------------------
// FbxBindingStrategy — UDP metadata keys; no native GUID through assimp
// ---------------------------------------------------------------------------

IBindingStrategy::Capability FbxBindingStrategy::Describe() const
{
    Capability c;
    c.format = "FBX";
    c.nativeGuid = false;   // assimp does not expose FBX internal UniqueId
    c.idSource = "user-defined properties on aiNode::mMetaData (--bim-id-property); fallback objectName";
    c.sceneMetadata = true;
    c.caveats = "duplicate node names get 3-digit dedup suffix (Wall -> Wall001); "
                "no stable GUID unless the exporter baked one into UDP";
    return c;
}

IBindingStrategy::IdResolution FbxBindingStrategy::ResolveObjectId(const MeshInstance& inst) const
{
    IdResolution r;
    auto row = MetadataRow(inst, /*inheritAncestors=*/false);
    if (row)
    {
        std::string v = LookupKeys(*row, DefaultIdKeys());
        if (!v.empty())
        {
            r.objectId = v;
            r.source = IdResolution::Source::MetadataKey;
            return r;
        }
    }
    // Fallback: objectName (weak — not stable across re-exports)
    if (!inst.objectName.empty())
    {
        r.objectId = inst.objectName;
        r.source = IdResolution::Source::ObjectName;
    }
    return r;
}

// ---------------------------------------------------------------------------
// GltfBindingStrategy — extras on metadata
// ---------------------------------------------------------------------------

IBindingStrategy::Capability GltfBindingStrategy::Describe() const
{
    Capability c;
    c.format = "glTF2";
    c.nativeGuid = false;
    c.idSource = "extras keys on aiNode::mMetaData (--bim-id-property); fallback objectName";
    c.sceneMetadata = true;
    c.caveats = "extras are exporter-specific; no standard ID key";
    return c;
}

IBindingStrategy::IdResolution GltfBindingStrategy::ResolveObjectId(const MeshInstance& inst) const
{
    IdResolution r;
    auto row = MetadataRow(inst, /*inheritAncestors=*/false);
    if (row)
    {
        std::string v = LookupKeys(*row, DefaultIdKeys());
        if (!v.empty())
        {
            r.objectId = v;
            r.source = IdResolution::Source::MetadataKey;
            return r;
        }
    }
    if (!inst.objectName.empty())
    {
        r.objectId = inst.objectName;
        r.source = IdResolution::Source::ObjectName;
    }
    return r;
}

// ---------------------------------------------------------------------------
// ObjBindingStrategy — names only
// ---------------------------------------------------------------------------

IBindingStrategy::Capability ObjBindingStrategy::Describe() const
{
    Capability c;
    c.format = "OBJ";
    c.nativeGuid = false;
    c.idSource = "objectName (node name) — no native ID concept";
    c.sceneMetadata = false;
    c.caveats = "join sidecar by name; empty row otherwise";
    return c;
}

IBindingStrategy::IdResolution ObjBindingStrategy::ResolveObjectId(const MeshInstance& inst) const
{
    IdResolution r;
    if (!inst.objectName.empty())
    {
        r.objectId = inst.objectName;
        r.source = IdResolution::Source::ObjectName;
    }
    return r;
}

// ---------------------------------------------------------------------------
// ThreeDsBindingStrategy — like OBJ plus $$$DUMMY names
// ---------------------------------------------------------------------------

IBindingStrategy::Capability ThreeDsBindingStrategy::Describe() const
{
    Capability c;
    c.format = "3DS";
    c.nativeGuid = false;
    c.idSource = "objectName / meshName — no native ID concept, no metadata";
    c.sceneMetadata = false;
    c.caveats = "placeholder nodes may be named \"$$$DUMMY\"; meshName often better than objectName";
    return c;
}

IBindingStrategy::IdResolution ThreeDsBindingStrategy::ResolveObjectId(const MeshInstance& inst) const
{
    IdResolution r;
    const std::string& name = (inst.objectName.empty() || inst.objectName == "$$$DUMMY")
                                  ? inst.meshName : inst.objectName;
    if (!name.empty())
    {
        r.objectId = name;
        r.source = IdResolution::Source::ObjectName;
    }
    return r;
}

// ---------------------------------------------------------------------------
// GenericBindingStrategy — fallback
// ---------------------------------------------------------------------------

IBindingStrategy::Capability GenericBindingStrategy::Describe() const
{
    Capability c;
    c.format = "Generic";
    c.nativeGuid = false;
    c.idSource = "aiNode::mMetaData keys, fallback objectName";
    c.sceneMetadata = true;
    c.caveats = "unrecognized format: generic metadata + name handling";
    return c;
}

IBindingStrategy::IdResolution GenericBindingStrategy::ResolveObjectId(const MeshInstance& inst) const
{
    IdResolution r;
    auto row = MetadataRow(inst, /*inheritAncestors=*/false);
    if (row)
    {
        std::string v = LookupKeys(*row, DefaultIdKeys());
        if (!v.empty())
        {
            r.objectId = v;
            r.source = IdResolution::Source::MetadataKey;
            return r;
        }
    }
    if (!inst.objectName.empty())
    {
        r.objectId = inst.objectName;
        r.source = IdResolution::Source::ObjectName;
    }
    return r;
}

// ---------------------------------------------------------------------------
// BindingStrategyRegistry
// ---------------------------------------------------------------------------

namespace
{
    IfcBindingStrategy      g_ifc;
    FbxBindingStrategy      g_fbx;
    GltfBindingStrategy     g_gltf;
    ObjBindingStrategy      g_obj;
    ThreeDsBindingStrategy  g_3ds;
    GenericBindingStrategy  g_generic;

    std::string ToLower(std::string s)
    {
        for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        return s;
    }

    // Normalize "Autodesk FBX Importer" / ".ifc" / "IFC" / "/path/model.obj"
    // -> "ifc"/"fbx"/"gltf2"/"obj"/"3ds".
    // Matching is EXTENSION/BASENAME-first (review #14): a plain substring
    // scan let "/data/ifc_archive/model.fbx" hit the "ifc" rule and report
    // the FBX model as IFC. Importer names ("Autodesk FBX Importer",
    // "glTF2Exporter", "OBJExporter") still resolve through the tail rules.
    std::string NormalizeFormat(const std::string& raw)
    {
        std::string s = ToLower(raw);

        // 1. File extension of the basename.
        size_t sep = s.find_last_of("/\\");
        std::string base = (sep == std::string::npos) ? s : s.substr(sep + 1);
        size_t dot = base.find_last_of('.');
        if (dot != std::string::npos && dot > 0 && dot + 1 < base.size())
        {
            const std::string ext = base.substr(dot + 1);
            if (ext == "ifc") return "ifc";
            if (ext == "fbx") return "fbx";
            if (ext == "gltf" || ext == "glb") return "gltf2";
            if (ext == "obj") return "obj";
            if (ext == "3ds" || ext == "max") return "3ds";
        }

        // 2. Whole-token match ("ifc", ".ifc", "gltf2", importer short ids).
        while (!base.empty() && base.front() == '.') base.erase(0, 1);
        if (base == "ifc") return "ifc";
        if (base == "fbx") return "fbx";
        if (base == "gltf" || base == "glb" || base == "gltf2") return "gltf2";
        if (base == "obj") return "obj";
        if (base == "3ds" || base == "max") return "3ds";

        // 3. Importer-name substrings, most specific first.
        if (s.find("fbx") != std::string::npos) return "fbx";
        if (s.find("ifc") != std::string::npos) return "ifc";
        if (s.find("gltf") != std::string::npos || s.find("glb") != std::string::npos) return "gltf2";
        if (s.find("obj") != std::string::npos) return "obj";
        if (s.find("3ds") != std::string::npos || s.find("max") != std::string::npos) return "3ds";
        return s;
    }
}

const IBindingStrategy& BindingStrategyRegistry::For(const std::string& assimpFormatId)
{
    const std::string f = NormalizeFormat(assimpFormatId);
    if (f == "ifc")    return g_ifc;
    if (f == "fbx")    return g_fbx;
    if (f == "gltf2")  return g_gltf;
    if (f == "obj")    return g_obj;
    if (f == "3ds")    return g_3ds;
    return g_generic;
}

std::vector<const IBindingStrategy*> BindingStrategyRegistry::All()
{
    return { &g_ifc, &g_fbx, &g_gltf, &g_obj, &g_3ds, &g_generic };
}
