// Copyright Johnlyon
//
// TileBuilder — modular 3D Tiles builder
//
// Migrated from TilesConverter.cpp. Provides:
//   - GlbBuilder:      binary glTF (.glb) construction
//   - B3dmBuilder:     Batched 3D Model (.b3dm) wrapping
//   - MaterialGrouper: per-cell material grouping and merging
//   - BBoxUtils:       bounding box helpers
//   - TilesetWriter:   tileset.json generation + tile writing
//

#include "TileBuilder.h"
#include "../MeshProjectionErrorCorrector/CoordinateTransform.hpp"
#include <assimp/scene.h>
#include <assimp/mesh.h>
#include <assimp/material.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cfloat>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include "../MeshProjectionErrorCorrector/Log.hpp"
#include <map>
#include <sstream>
#include <unordered_map>
#include <vector>
#include <jpeglib.h>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;

// ===========================================================================
// Binary constants & helpers
// ===========================================================================
namespace
{
    constexpr uint32_t B3DM_MAGIC = 0x6D643362;  // "b3dm" LE
    constexpr uint32_t B3DM_VERSION = 1;

    template<typename T>
    void writeLE(std::vector<uint8_t>& buf, T value)
    {
        for (size_t i = 0; i < sizeof(T); ++i)
            buf.push_back(static_cast<uint8_t>((value >> (i * 8)) & 0xFF));
    }

    void writeBytes(std::vector<uint8_t>& buf, const void* src, size_t len)
    {
        const auto* p = static_cast<const uint8_t*>(src);
        buf.insert(buf.end(), p, p + len);
    }

    // aiMatrix4x4 helpers
    aiMatrix4x4 toMatrix4x4(const float* m)
    {
        aiMatrix4x4 mat;
        mat.a1 = m[0];  mat.a2 = m[1];  mat.a3 = m[2];  mat.a4 = m[3];
        mat.b1 = m[4];  mat.b2 = m[5];  mat.b3 = m[6];  mat.b4 = m[7];
        mat.c1 = m[8];  mat.c2 = m[9];  mat.c3 = m[10]; mat.c4 = m[11];
        mat.d1 = m[12]; mat.d2 = m[13]; mat.d3 = m[14]; mat.d4 = m[15];
        return mat;
    }

    std::string levelDir(const GridCell& cell)
    {
        return cell.isOverflow ? "overflow" : ("L" + std::to_string(cell.level));
    }

    // Encode tightly-packed RGBA8 pixels to an in-memory JPEG buffer.
    // Photogrammetry textures are photographic, so lossy JPEG matches the
    // source OSGB compression and is far smaller than lossless PNG.
    bool EncodeJPEG(const uint8_t* rgba, int w, int h, std::vector<uint8_t>& out,
                    int quality = 95)
    {
        out.clear();
        if (!rgba || w <= 0 || h <= 0) return false;

        struct jpeg_compress_struct cinfo;
        struct jpeg_error_mgr jerr;
        cinfo.err = jpeg_std_error(&jerr);
        jpeg_create_compress(&cinfo);

        unsigned char* outbuf = nullptr;
        unsigned long outsize = 0;
        jpeg_mem_dest(&cinfo, &outbuf, &outsize);

        cinfo.image_width = w;
        cinfo.image_height = h;
        cinfo.input_components = 3;
        cinfo.in_color_space = JCS_RGB;
        jpeg_set_defaults(&cinfo);
        jpeg_set_quality(&cinfo, quality, TRUE);
        jpeg_start_compress(&cinfo, TRUE);

        std::vector<uint8_t> scanline(static_cast<size_t>(w) * 3);
        while (cinfo.next_scanline < cinfo.image_height)
        {
            const uint8_t* src = rgba + static_cast<size_t>(cinfo.next_scanline) * w * 4;
            for (int x = 0; x < w; ++x)
            {
                scanline[x * 3 + 0] = src[x * 4 + 0];
                scanline[x * 3 + 1] = src[x * 4 + 1];
                scanline[x * 3 + 2] = src[x * 4 + 2];
            }
            JSAMPROW row = scanline.data();
            jpeg_write_scanlines(&cinfo, &row, 1);
        }
        jpeg_finish_compress(&cinfo);

        out.assign(outbuf, outbuf + outsize);
        if (outbuf) free(outbuf);
        jpeg_destroy_compress(&cinfo);
        return !out.empty();
    }
}

// ===========================================================================
// GlbBuilder::Build — binary glTF construction
// ===========================================================================

bool GlbBuilder::Build(const std::vector<MergedMeshGroup>& groups,
                       BinaryBlob& outGlb,
                       const std::string& textureBaseDir,
                       bool doubleSided)
{
    outGlb.data.clear();
    if (groups.empty()) return false;

    // ------------------------------------------------------------------
    // Binary buffer layout (per primitive):
    //   positions  — FLOAT VEC3, count = vertexCount
    //   normals    — FLOAT VEC3, count = vertexCount
    //   texcoords  — FLOAT VEC2, count = vertexCount (even if all zeros)
    //   indices    — UNSIGNED_INT SCALAR, count = indexCount
    // Followed by embedded image bytes (one per unique texture).
    // ------------------------------------------------------------------

    struct PrimLayout {
        size_t posByteOffset, posByteLength;
        size_t normByteOffset, normByteLength;
        size_t uvByteOffset, uvByteLength;
        size_t idxByteOffset, idxByteLength;
        size_t vertexCount, indexCount;
        uint32_t indexMax;
        float posMin[3], posMax[3];
    };
    std::vector<PrimLayout> layouts;
    layouts.reserve(groups.size());

    // Collect unique texture paths
    std::vector<std::string> uniqueTexPaths;
    std::vector<int> groupTexIndex(groups.size(), -1);

    for (size_t gi = 0; gi < groups.size(); ++gi) {
        const auto& tp = groups[gi].diffuseTexturePath;
        if (tp.empty()) continue;
        if (!tp.empty() && tp[0] == '*') continue;

        auto it = std::find(uniqueTexPaths.begin(), uniqueTexPaths.end(), tp);
        if (it == uniqueTexPaths.end()) {
            groupTexIndex[gi] = static_cast<int>(uniqueTexPaths.size());
            uniqueTexPaths.push_back(tp);
        } else {
            groupTexIndex[gi] = static_cast<int>(it - uniqueTexPaths.begin());
        }
    }

    // Map each unique texture path to a representative group carrying
    // embedded pixel data (ContextCapture OSGB embeds textures inline).
    std::vector<int> texPathGroup(uniqueTexPaths.size(), -1);
    for (size_t gi = 0; gi < groups.size(); ++gi) {
        int ti = groupTexIndex[gi];
        if (ti >= 0 && texPathGroup[ti] < 0 && !groups[gi].texturePixels.empty())
            texPathGroup[ti] = static_cast<int>(gi);
    }

    std::vector<uint8_t> bin;
    std::vector<size_t> layoutToGroup;  // map layout index → original group index
    for (size_t gi = 0; gi < groups.size(); ++gi) {
        auto& g = groups[gi];
        // Skip empty primitives: < 3 vertices, or no indices (all faces
        // degenerate/non-triangular). A zero-count indices accessor is
        // invalid glTF and renders nothing.
        if (g.vertexCount() < 3 || g.indexCount() == 0) continue;

        PrimLayout L;
        L.vertexCount = g.vertexCount();
        L.indexCount  = g.indexCount();

        L.posMin[0]=L.posMin[1]=L.posMin[2]=FLT_MAX;
        L.posMax[0]=L.posMax[1]=L.posMax[2]=-FLT_MAX;
        for (size_t i=0; i<L.vertexCount; ++i) {
            float x=g.positions[i*3], y=g.positions[i*3+1], z=g.positions[i*3+2];
            if (x<L.posMin[0]) L.posMin[0]=x; if (x>L.posMax[0]) L.posMax[0]=x;
            if (y<L.posMin[1]) L.posMin[1]=y; if (y>L.posMax[1]) L.posMax[1]=y;
            if (z<L.posMin[2]) L.posMin[2]=z; if (z>L.posMax[2]) L.posMax[2]=z;
        }

        L.indexMax = 0;
        for (size_t i=0; i<L.indexCount; ++i)
            if (g.indices[i] > L.indexMax) L.indexMax = g.indices[i];

        L.posByteOffset = bin.size();
        L.posByteLength = L.vertexCount * 3 * sizeof(float);
        writeBytes(bin, g.positions.data(), L.posByteLength);

        L.normByteOffset = bin.size();
        L.normByteLength = L.vertexCount * 3 * sizeof(float);
        writeBytes(bin, g.normals.data(), L.normByteLength);

        L.uvByteOffset = bin.size();
        L.uvByteLength = L.vertexCount * 2 * sizeof(float);
        writeBytes(bin, g.texcoords.data(), L.uvByteLength);

        L.idxByteOffset = bin.size();
        L.idxByteLength = L.indexCount * sizeof(uint32_t);
        writeBytes(bin, g.indices.data(), L.idxByteLength);

        layouts.push_back(L);
        layoutToGroup.push_back(gi);
    }

    if (layouts.empty()) return false;

    // ------------------------------------------------------------------
    // Load and embed texture files
    // ------------------------------------------------------------------
    struct ImageInfo {
        size_t byteOffset;
        size_t byteLength;
        std::string mimeType;
    };
    std::vector<ImageInfo> images;
    images.reserve(uniqueTexPaths.size());

    for (size_t ti = 0; ti < uniqueTexPaths.size(); ++ti) {
        const auto& texPath = uniqueTexPaths[ti];

        // Embedded texture pixels take priority over the (virtual) file path:
        // ContextCapture OSGB embeds textures inline and the filename reported
        // by osg::Image has no counterpart on disk. Encode to JPEG in-memory
        // and append the bytes to the GLB buffer.
        int repGroup = texPathGroup[ti];
        if (repGroup >= 0) {
            const auto& g = groups[repGroup];
            std::vector<uint8_t> jpg;
            if (EncodeJPEG(g.texturePixels.data(), g.textureWidth, g.textureHeight, jpg)) {
                ImageInfo info;
                info.byteOffset = bin.size();
                info.byteLength = jpg.size();
                info.mimeType = "image/jpeg";
                bin.insert(bin.end(), jpg.begin(), jpg.end());
                images.push_back(info);
                continue;
            }
        }

        std::string resolvedPath;
#ifdef _WIN32
        if (texPath.size() >= 2 && texPath[1] == ':')
            resolvedPath = texPath;
        else
#else
        if (!texPath.empty() && texPath[0] == '/')
            resolvedPath = texPath;
        else
#endif
        {
            if (!textureBaseDir.empty())
                resolvedPath = textureBaseDir + "/" + texPath;
            else
                resolvedPath = texPath;
        }

        std::ifstream file(resolvedPath, std::ios::binary | std::ios::ate);
        if (!file.is_open()) {
            std::string fname = texPath;
            auto slashPos = fname.find_last_of("/\\");
            if (slashPos != std::string::npos)
                fname = fname.substr(slashPos + 1);
            std::string altPath = textureBaseDir.empty()
                ? fname : textureBaseDir + "/" + fname;
            file.open(altPath, std::ios::binary | std::ios::ate);
        }
        if (!file.is_open()) {
            // Texture file missing: embed a 1x1 white PNG so the glTF remains
            // valid. CesiumJS rejects images with an empty MIME type, so an
            // empty placeholder would abort loading the whole tile. The
            // material still references this texture and renders white.
            static const unsigned char kWhitePng[] = {
                0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A, 0x00, 0x00, 0x00, 0x0D,
                0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01,
                0x08, 0x06, 0x00, 0x00, 0x00, 0x1F, 0x15, 0xC4, 0x89, 0x00, 0x00, 0x00,
                0x0D, 0x49, 0x44, 0x41, 0x54, 0x78, 0x9C, 0x63, 0xF8, 0xFF, 0xFF, 0xFF,
                0x7F, 0x00, 0x09, 0xFB, 0x03, 0xFD, 0x2A, 0x86, 0xE3, 0x8A, 0x00, 0x00,
                0x00, 0x00, 0x49, 0x45, 0x4E, 0x44, 0xAE, 0x42, 0x60, 0x82
            };
            ImageInfo info;
            info.byteOffset = bin.size();
            info.byteLength = sizeof(kWhitePng);
            info.mimeType = "image/png";
            bin.insert(bin.end(), kWhitePng, kWhitePng + sizeof(kWhitePng));
            images.push_back(info);
            continue;
        }

        size_t fileSize = static_cast<size_t>(file.tellg());
        file.seekg(0, std::ios::beg);

        ImageInfo info;
        info.byteOffset = bin.size();
        info.byteLength = fileSize;

        std::string lower = texPath;
        std::transform(lower.begin(), lower.end(), lower.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        if (lower.find(".png") != std::string::npos)
            info.mimeType = "image/png";
        else if (lower.find(".jpg") != std::string::npos ||
                 lower.find(".jpeg") != std::string::npos)
            info.mimeType = "image/jpeg";
        else
            info.mimeType = "image/png";

        size_t oldSize = bin.size();
        bin.resize(oldSize + fileSize);
        file.read(reinterpret_cast<char*>(bin.data() + oldSize), fileSize);
        images.push_back(info);
    }

    size_t imageBVStart = 4 * layouts.size();

    // ------------------------------------------------------------------
    // Build glTF JSON string
    // ------------------------------------------------------------------
    std::ostringstream json;
    json.precision(8);
    json << std::fixed;

    json << "{\"asset\":{\"version\":\"2.0\"},\"scene\":0";

    // scenes
    json << ",\"scenes\":[{\"nodes\":[";
    for (size_t i=0; i<layouts.size(); ++i) {
        if (i) json << ',';
        json << i;
    }
    json << "]}]";

    // nodes — identity. CesiumJS handles Y-up→Z-up conversion automatically.
    json << ",\"nodes\":[";
    for (size_t i=0; i<layouts.size(); ++i) {
        if (i) json << ',';
        json << "{\"mesh\":" << i << '}';
    }
    json << ']';

    // bufferViews
    json << ",\"bufferViews\":[";
    for (size_t pi=0; pi<layouts.size(); ++pi) {
        auto& L = layouts[pi];
        std::string comma = (pi==0) ? "" : ",";

        json << comma << "{\"buffer\":0,\"byteOffset\":" << L.posByteOffset
             << ",\"byteLength\":" << L.posByteLength << ",\"target\":34962}";
        json << ",{\"buffer\":0,\"byteOffset\":" << L.normByteOffset
             << ",\"byteLength\":" << L.normByteLength << ",\"target\":34962}";
        json << ",{\"buffer\":0,\"byteOffset\":" << L.uvByteOffset
             << ",\"byteLength\":" << L.uvByteLength << ",\"target\":34962}";
        json << ",{\"buffer\":0,\"byteOffset\":" << L.idxByteOffset
             << ",\"byteLength\":" << L.idxByteLength << ",\"target\":34963}";
    }
    for (size_t ii = 0; ii < images.size(); ++ii) {
        json << ",{\"buffer\":0,\"byteOffset\":" << images[ii].byteOffset
             << ",\"byteLength\":" << images[ii].byteLength << "}";
    }
    json << ']';

    // accessors
    json << ",\"accessors\":[";
    for (size_t pi=0; pi<layouts.size(); ++pi) {
        auto& L = layouts[pi];
        if (pi) json << ',';
        size_t bvBase = pi * 4;

        json << "{\"bufferView\":" << bvBase
             << ",\"componentType\":5126,\"count\":" << L.vertexCount
             << ",\"type\":\"VEC3\",\"min\":[" << L.posMin[0] << ',' << L.posMin[1] << ',' << L.posMin[2]
             << "],\"max\":[" << L.posMax[0] << ',' << L.posMax[1] << ',' << L.posMax[2] << "]}";
        json << ",{\"bufferView\":" << (bvBase+1)
             << ",\"componentType\":5126,\"count\":" << L.vertexCount
             << ",\"type\":\"VEC3\"}";
        json << ",{\"bufferView\":" << (bvBase+2)
             << ",\"componentType\":5126,\"count\":" << L.vertexCount
             << ",\"type\":\"VEC2\"}";
        json << ",{\"bufferView\":" << (bvBase+3)
             << ",\"componentType\":5125,\"count\":" << L.indexCount
             << ",\"type\":\"SCALAR\"}";
    }
    json << ']';

    // buffer
    json << ",\"buffers\":[{\"byteLength\":" << bin.size() << "}]";

    // samplers
    json << ",\"samplers\":[{\"magFilter\":9729,\"minFilter\":9986,"
         << "\"wrapS\":33071,\"wrapT\":33071}]";

    // images
    json << ",\"images\":[";
    for (size_t ii = 0; ii < images.size(); ++ii) {
        if (ii) json << ',';
        json << "{\"bufferView\":" << (imageBVStart + ii)
             << ",\"mimeType\":\"" << images[ii].mimeType << "\"}";
    }
    json << ']';

    // textures
    json << ",\"textures\":[";
    for (size_t ii = 0; ii < images.size(); ++ii) {
        if (ii) json << ',';
        json << "{\"source\":" << ii << ",\"sampler\":0}";
    }
    json << ']';

    // meshes
    json << ",\"meshes\":[";
    for (size_t pi=0; pi<layouts.size(); ++pi) {
        if (pi) json << ',';
        size_t accBase = pi * 4;
        json << "{\"primitives\":[{\"attributes\":{"
             << "\"POSITION\":" << accBase
             << ",\"NORMAL\":" << (accBase+1)
             << ",\"TEXCOORD_0\":" << (accBase+2)
             << "},\"indices\":" << (accBase+3)
             << ",\"material\":" << pi
             << "}]}";
    }
    json << ']';

    // materials
    json << ",\"materials\":[";
    for (size_t pi=0; pi<layouts.size(); ++pi) {
        size_t gi = layoutToGroup[pi];
        if (pi) json << ',';
        json << "{\"pbrMetallicRoughness\":{"
             << "\"baseColorFactor\":["
             << groups[gi].baseColorFactor[0] << ',' << groups[gi].baseColorFactor[1] << ','
             << groups[gi].baseColorFactor[2] << ',' << groups[gi].baseColorFactor[3]
             << "],\"metallicFactor\":0.0,\"roughnessFactor\":1.0";

        int texIdx = groupTexIndex[gi];
        if (texIdx >= 0 && texIdx < static_cast<int>(images.size()) &&
            images[texIdx].byteLength > 0) {
            json << ",\"baseColorTexture\":{\"index\":" << texIdx << '}';
        }
        json << "},\"doubleSided\":" << (doubleSided ? "true" : "false") << "}";
    }
    json << ']';

    json << '}';

    std::string jsonStr = json.str();

    // Pad JSON to 4-byte alignment
    while (jsonStr.size() % 4 != 0)
        jsonStr.push_back(' ');

    // ------------------------------------------------------------------
    // Assemble GLB
    // ------------------------------------------------------------------
    uint32_t jsonLen = static_cast<uint32_t>(jsonStr.size());
    uint32_t binLen  = static_cast<uint32_t>(bin.size());
    uint32_t binChunkLen = binLen;
    while (binChunkLen % 4 != 0) binChunkLen++;
    uint32_t totalLen = 12 + 8 + jsonLen + 8 + binChunkLen;

    outGlb.data.resize(totalLen);
    uint8_t* dst = outGlb.data.data();

    // GLB header
    *reinterpret_cast<uint32_t*>(dst)     = 0x46546C67;
    *reinterpret_cast<uint32_t*>(dst + 4) = 2;
    *reinterpret_cast<uint32_t*>(dst + 8) = totalLen;

    // JSON chunk
    *reinterpret_cast<uint32_t*>(dst + 12) = jsonLen;
    *reinterpret_cast<uint32_t*>(dst + 16) = 0x4E4F534A;
    std::memcpy(dst + 20, jsonStr.data(), jsonLen);

    // BIN chunk
    uint32_t binHdrOff = 20 + jsonLen;
    *reinterpret_cast<uint32_t*>(dst + binHdrOff)     = binChunkLen;
    *reinterpret_cast<uint32_t*>(dst + binHdrOff + 4) = 0x004E4942;
    std::memcpy(dst + binHdrOff + 8, bin.data(), binLen);
    for (uint32_t i = binLen; i < binChunkLen; ++i)
        dst[binHdrOff + 8 + i] = 0;

    return true;
}

// ===========================================================================
// B3dmBuilder — b3dm wrapping
// ===========================================================================

bool B3dmBuilder::Build(const BinaryBlob& glb, BinaryBlob& outB3dm)
{
    outB3dm.data.clear();

    std::string ftJson = "{\"BATCH_LENGTH\":0}  "; // 20 bytes: 18 JSON + 2 spaces

    constexpr size_t HEADER_SIZE = 28;
    size_t ftJsonLen = ftJson.size();    // 20
    size_t glbOffset = HEADER_SIZE + ftJsonLen; // 48

    size_t totalLenNoPad = glbOffset + glb.size();
    size_t trailingPad = (8 - (totalLenNoPad % 8)) % 8;
    size_t totalLen = totalLenNoPad + trailingPad;

    outB3dm.data.reserve(totalLen);

    writeLE(outB3dm.data, B3DM_MAGIC);
    writeLE(outB3dm.data, B3DM_VERSION);
    writeLE<uint32_t>(outB3dm.data, static_cast<uint32_t>(totalLen));
    writeLE<uint32_t>(outB3dm.data, static_cast<uint32_t>(ftJsonLen));
    writeLE<uint32_t>(outB3dm.data, 0); // ftBinLen = 0
    writeLE<uint32_t>(outB3dm.data, 0); // btJsonLen = 0
    writeLE<uint32_t>(outB3dm.data, 0); // btBinLen = 0

    writeBytes(outB3dm.data, ftJson.data(), ftJsonLen);
    writeBytes(outB3dm.data, glb.ptr(), glb.size());
    while (outB3dm.data.size() % 8 != 0)
        outB3dm.data.push_back(0);

    return true;
}

void B3dmBuilder::PadTo8(std::vector<uint8_t>& buf)
{
    while (buf.size() % 8 != 0)
        buf.push_back(0);
}

// ===========================================================================
// MaterialGrouper — per-cell material grouping and merging
// ===========================================================================

void MaterialGrouper::MergeGroupsByMaterial(std::vector<MergedMeshGroup>& groups)
{
    if (groups.size() <= 1) return;

    auto makeKey = [](const MergedMeshGroup& g) -> std::string {
        char buf[256];
        snprintf(buf, sizeof(buf), "%08x,%08x,%08x,%08x|%s",
            *reinterpret_cast<const uint32_t*>(&g.baseColorFactor[0]),
            *reinterpret_cast<const uint32_t*>(&g.baseColorFactor[1]),
            *reinterpret_cast<const uint32_t*>(&g.baseColorFactor[2]),
            *reinterpret_cast<const uint32_t*>(&g.baseColorFactor[3]),
            g.diffuseTexturePath.c_str());
        return std::string(buf);
    };

    std::map<std::string, std::vector<size_t>> keyGroups;
    for (size_t i = 0; i < groups.size(); ++i)
        keyGroups[makeKey(groups[i])].push_back(i);

    if (keyGroups.size() == groups.size()) return;

    std::vector<MergedMeshGroup> merged;
    merged.reserve(keyGroups.size());

    for (auto& kv : keyGroups)
    {
        auto& indices = kv.second;
        if (indices.size() == 1)
        {
            merged.push_back(std::move(groups[indices[0]]));
        }
        else
        {
            MergedMeshGroup combined;
            combined.materialIndex = groups[indices[0]].materialIndex;
            combined.baseColorFactor = groups[indices[0]].baseColorFactor;
            combined.diffuseTexturePath = groups[indices[0]].diffuseTexturePath;

            size_t totalVerts = 0;
            size_t totalIndices = 0;
            for (auto idx : indices)
            {
                totalVerts += groups[idx].vertexCount();
                totalIndices += groups[idx].indexCount();
            }

            combined.positions.reserve(totalVerts * 3);
            combined.normals.reserve(totalVerts * 3);
            combined.texcoords.reserve(totalVerts * 2);
            combined.indices.reserve(totalIndices);

            for (int a = 0; a < 3; ++a)
            {
                combined.bboxMin[a] = DBL_MAX;
                combined.bboxMax[a] = -DBL_MAX;
            }

            for (auto idx : indices)
            {
                auto& g = groups[idx];
                uint32_t baseVertex = static_cast<uint32_t>(combined.vertexCount());

                combined.positions.insert(combined.positions.end(),
                    g.positions.begin(), g.positions.end());
                combined.normals.insert(combined.normals.end(),
                    g.normals.begin(), g.normals.end());
                if (!g.texcoords.empty())
                    combined.texcoords.insert(combined.texcoords.end(),
                        g.texcoords.begin(), g.texcoords.end());
                else
                    combined.texcoords.resize(combined.vertexCount() * 2, 0.0f);

                for (auto idxVal : g.indices)
                    combined.indices.push_back(baseVertex + idxVal);

                for (int a = 0; a < 3; ++a)
                {
                    if (g.bboxMin[a] < combined.bboxMin[a])
                        combined.bboxMin[a] = g.bboxMin[a];
                    if (g.bboxMax[a] > combined.bboxMax[a])
                        combined.bboxMax[a] = g.bboxMax[a];
                }
            }

            merged.push_back(std::move(combined));
        }
    }

    groups = std::move(merged);
}

void MaterialGrouper::GroupCellByMaterial(GridCell& cell, const aiScene* scene,
                                          const TileBuildOptions& opts)
{
    if (cell.instances.empty()) return;

    struct Accumulator {
        std::vector<float> positions;
        std::vector<float> normals;
        std::vector<float> texcoords;
        std::vector<uint32_t> indices;
        double bboxMin[3] = { DBL_MAX, DBL_MAX, DBL_MAX };
        double bboxMax[3] = { -DBL_MAX, -DBL_MAX, -DBL_MAX };
        double localBboxMin[3] = { DBL_MAX, DBL_MAX, DBL_MAX };
        double localBboxMax[3] = { -DBL_MAX, -DBL_MAX, -DBL_MAX };
        bool hasUV = false;
        bool hasNormal = false;
        size_t degenerateSkipped = 0;
    };
    std::map<int, Accumulator> accMap;

    for (auto& inst : cell.instances)
    {
        const aiMesh* mesh = scene->mMeshes[inst.meshIndex];
        int matIdx = mesh->mMaterialIndex;
        auto& acc = accMap[matIdx];
        uint32_t baseVertex = static_cast<uint32_t>(acc.positions.size() / 3);
        aiMatrix4x4 world = toMatrix4x4(inst.worldTransform);

        bool meshHasUV = mesh->HasTextureCoords(0);
        bool meshHasNormal = mesh->HasNormals();

        // Pre-check: skip mesh instances with NaN vertices
        {
            bool hasNaN = false;
            for (unsigned int vi = 0; vi < mesh->mNumVertices; ++vi)
            {
                aiVector3D wp = world * mesh->mVertices[vi];
                if (std::isnan(wp.x) || std::isnan(wp.y) || std::isnan(wp.z))
                {
                    hasNaN = true;
                    break;
                }
            }
            if (hasNaN)
            {
                static int nanSkipCount = 0;
                if (++nanSkipCount <= 20)
                    MGO_LOG(Warning) << "[TileBuilder] skipping mesh "
                              << inst.meshIndex << " (material " << matIdx
                              << ") due to NaN vertices";
                continue;
            }
        }

        // Normal matrix: M_normal = (M_world^{-1})^T, computed once per instance.
        aiMatrix4x4 normalMat;
        if (meshHasNormal)
        {
            normalMat = world;
            normalMat.a4 = normalMat.b4 = normalMat.c4 = 0;
            normalMat.d1 = normalMat.d2 = normalMat.d3 = 0;
            normalMat.d4 = 1;
            normalMat.Inverse();
            normalMat.Transpose();
        }

        for (unsigned int vi = 0; vi < mesh->mNumVertices; ++vi)
        {
            aiVector3D wp = world * mesh->mVertices[vi];
            double wx = wp.x, wy = wp.y, wz = wp.z;

            // Z-up → Assimp Y-up conversion
            if (opts.inputIsZUp)
            {
                double zup_in[3] = {wx, wy, wz};
                double ayu_out[3];
                MGO::CoordinateTransform::Convert(zup_in, MGO::CoordinateFrame::ZUp,
                                                  ayu_out, MGO::CoordinateFrame::AssimpYUp);
                wx = ayu_out[0]; wy = ayu_out[1]; wz = ayu_out[2];
            }

            // Track local bbox
            if (wx < acc.localBboxMin[0]) acc.localBboxMin[0] = wx;
            if (wy < acc.localBboxMin[1]) acc.localBboxMin[1] = wy;
            if (wz < acc.localBboxMin[2]) acc.localBboxMin[2] = wz;
            if (wx > acc.localBboxMax[0]) acc.localBboxMax[0] = wx;
            if (wy > acc.localBboxMax[1]) acc.localBboxMax[1] = wy;
            if (wz > acc.localBboxMax[2]) acc.localBboxMax[2] = wz;

            acc.positions.push_back(static_cast<float>(wx));
            acc.positions.push_back(static_cast<float>(wy));
            acc.positions.push_back(static_cast<float>(wz));

            if (wx < acc.bboxMin[0]) acc.bboxMin[0] = wx;
            if (wy < acc.bboxMin[1]) acc.bboxMin[1] = wy;
            if (wz < acc.bboxMin[2]) acc.bboxMin[2] = wz;
            if (wx > acc.bboxMax[0]) acc.bboxMax[0] = wx;
            if (wy > acc.bboxMax[1]) acc.bboxMax[1] = wy;
            if (wz > acc.bboxMax[2]) acc.bboxMax[2] = wz;

            if (meshHasNormal)
            {
                aiVector3D wn = normalMat * mesh->mNormals[vi];
                wn.Normalize();
                if (opts.inputIsZUp)
                {
                    double zup_n[3] = {wn.x, wn.y, wn.z};
                    double ayu_n[3];
                    MGO::CoordinateTransform::ConvertNormal(zup_n, MGO::CoordinateFrame::ZUp,
                                                            ayu_n, MGO::CoordinateFrame::AssimpYUp);
                    wn.x = ayu_n[0]; wn.y = ayu_n[1]; wn.z = ayu_n[2];
                }
                acc.normals.push_back(wn.x);
                acc.normals.push_back(wn.y);
                acc.normals.push_back(wn.z);
                acc.hasNormal = true;
            }
            else
            {
                acc.normals.push_back(0.0f);
                acc.normals.push_back(0.0f);
                acc.normals.push_back(0.0f);
            }

            if (meshHasUV)
            {
                acc.texcoords.push_back(mesh->mTextureCoords[0][vi].x);
                acc.texcoords.push_back(mesh->mTextureCoords[0][vi].y);
                acc.hasUV = true;
            }
            else
            {
                acc.texcoords.push_back(0.0f);
                acc.texcoords.push_back(0.0f);
            }
        }

        for (unsigned int fi = 0; fi < mesh->mNumFaces; ++fi)
        {
            const aiFace& face = mesh->mFaces[fi];
            if (face.mNumIndices == 3)
            {
                unsigned int i0 = face.mIndices[0];
                unsigned int i1 = face.mIndices[1];
                unsigned int i2 = face.mIndices[2];

                if (i0 >= mesh->mNumVertices ||
                    i1 >= mesh->mNumVertices ||
                    i2 >= mesh->mNumVertices)
                    continue;

                if (i0 == i1 || i1 == i2 || i0 == i2)
                {
                    acc.degenerateSkipped++;
                    continue;
                }

                acc.indices.push_back(baseVertex + i0);
                acc.indices.push_back(baseVertex + i1);
                acc.indices.push_back(baseVertex + i2);
            }
        }
    }

    for (auto& kv : accMap)
    {
        auto& a = kv.second;

        // Empty primitive: vertices accumulated but no triangle survived (all
        // faces degenerate or non-triangular). Drop it here - a zero-index
        // group would build a glTF primitive whose indices accessor has
        // count 0, which glTF 2.0 forbids (count must be >= 1).
        if (a.indices.empty())
        {
            // Neutralize local bbox so the union below ignores these
            // unexported vertices.
            for (int ax = 0; ax < 3; ++ax)
            {
                a.localBboxMin[ax] = DBL_MAX;
                a.localBboxMax[ax] = -DBL_MAX;
            }
            continue;
        }

        size_t vc = a.positions.size() / 3;

        MergedMeshGroup g;
        g.materialIndex = kv.first;

        if (scene && (unsigned int)g.materialIndex < scene->mNumMaterials)
        {
            aiMaterial* mat = scene->mMaterials[g.materialIndex];
            aiColor4D diffuse;
            if (aiGetMaterialColor(mat, AI_MATKEY_COLOR_DIFFUSE, &diffuse) == AI_SUCCESS)
            {
                g.baseColorFactor[0] = diffuse.r;
                g.baseColorFactor[1] = diffuse.g;
                g.baseColorFactor[2] = diffuse.b;
                g.baseColorFactor[3] = diffuse.a;
            }

            aiString texPath;
            if (mat->GetTexture(aiTextureType_DIFFUSE, 0, &texPath) == AI_SUCCESS)
            {
                g.diffuseTexturePath = texPath.C_Str();
            }
        }

        g.positions = std::move(a.positions);
        g.indices   = std::move(a.indices);

        if (a.hasNormal)
            g.normals = std::move(a.normals);
        else
            g.normals.assign(vc * 3, 0.0f);

        if (a.hasUV)
            g.texcoords = std::move(a.texcoords);
        else
            g.texcoords.assign(vc * 2, 0.0f);

        for (int ax = 0; ax < 3; ++ax)
        {
            g.bboxMin[ax] = a.bboxMin[ax];
            g.bboxMax[ax] = a.bboxMax[ax];
        }

        cell.materialGroups.push_back(std::move(g));
    }

    MergeGroupsByMaterial(cell.materialGroups);

    // Skip near-empty tiles (< 50 output vertices)
    {
        size_t totalVerts = 0;
        for (auto& g : cell.materialGroups)
            totalVerts += g.vertexCount();
        if (totalVerts < 50)
        {
            cell.materialGroups.clear();
            cell.hasContent = false;
            return;
        }
    }

    // Update cell bbox from material groups
    if (!cell.materialGroups.empty())
    {
        for (int a = 0; a < 3; ++a)
        {
            cell.bboxMin[a] = DBL_MAX;
            cell.bboxMax[a] = -DBL_MAX;
            cell.localBboxMin[a] = DBL_MAX;
            cell.localBboxMax[a] = -DBL_MAX;
        }
        for (auto& g : cell.materialGroups)
        {
            for (int a = 0; a < 3; ++a)
            {
                if (g.bboxMin[a] < cell.bboxMin[a]) cell.bboxMin[a] = g.bboxMin[a];
                if (g.bboxMax[a] > cell.bboxMax[a]) cell.bboxMax[a] = g.bboxMax[a];
            }
        }
        // Union local bbox across all accumulators
        for (auto& kv : accMap)
        {
            auto& a = kv.second;
            for (int ax = 0; ax < 3; ++ax)
            {
                if (a.localBboxMin[ax] < cell.localBboxMin[ax]) cell.localBboxMin[ax] = a.localBboxMin[ax];
                if (a.localBboxMax[ax] > cell.localBboxMax[ax]) cell.localBboxMax[ax] = a.localBboxMax[ax];
            }
        }
    }
}

// ===========================================================================
// BBoxUtils — bounding box helpers
// ===========================================================================

nlohmann::ordered_json BBoxUtils::WriteBoxJson(const double* bmin, const double* bmax)
{
    // Convert from model Y-up (East,Up,North) to 3D Tiles Z-up.
    // GroupCellByMaterial handles Z-up→Y-up vertex conversion when inputIsZUp is set,
    // so bbox is always in Y-up by this point.
    double convMin[3], convMax[3];
    MGO::CoordinateTransform::ConvertBBox(bmin, bmax, MGO::CoordinateFrame::AssimpYUp,
                                          convMin, convMax, MGO::CoordinateFrame::TilesZUp);

    double cx = (convMin[0] + convMax[0]) * 0.5;
    double cy = (convMin[1] + convMax[1]) * 0.5;
    double cz = (convMin[2] + convMax[2]) * 0.5;
    double hx = (convMax[0] - convMin[0]) * 0.5;
    double hy = (convMax[1] - convMin[1]) * 0.5;
    double hz = (convMax[2] - convMin[2]) * 0.5;
    if (hx < 0.001) hx = 0.001;
    if (hy < 0.001) hy = 0.001;
    if (hz < 0.001) hz = 0.001;

    return {
        {"box", {cx, cy, cz, hx, 0.0, 0.0, 0.0, hy, 0.0, 0.0, 0.0, hz}}
    };
}

double BBoxUtils::Diagonal(const double* bmin, const double* bmax)
{
    double dx = bmax[0] - bmin[0];
    double dy = bmax[1] - bmin[1];
    double dz = bmax[2] - bmin[2];
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

void BBoxUtils::UpdateGridCellBBoxes(GridCell& cell)
{
    // Recurse to children first (bottom-up)
    for (auto& c : cell.children)
        if (c) UpdateGridCellBBoxes(*c);

    // Union children's bboxes into this cell.
    // This MUST run even for content cells: a cell can have both its own
    // instances (hasContent) AND children.
    bool first = !cell.hasContent;
    if (first)
    {
        for (auto& c : cell.children)
        {
            if (!c) continue;
            if (!c->hasContent && !c->hasChildren() && c->materialGroups.empty()) continue;
            for (int a = 0; a < 3; ++a)
            {
                cell.bboxMin[a] = c->bboxMin[a];
                cell.bboxMax[a] = c->bboxMax[a];
                cell.localBboxMin[a] = c->localBboxMin[a];
                cell.localBboxMax[a] = c->localBboxMax[a];
            }
            first = false;
            break;
        }
    }
    for (auto& c : cell.children)
    {
        if (!c) continue;
        if (!c->hasContent && !c->hasChildren() && c->materialGroups.empty()) continue;
        for (int a = 0; a < 3; ++a)
        {
            if (c->bboxMin[a] < cell.bboxMin[a]) cell.bboxMin[a] = c->bboxMin[a];
            if (c->bboxMax[a] > cell.bboxMax[a]) cell.bboxMax[a] = c->bboxMax[a];
            if (c->localBboxMin[a] < cell.localBboxMin[a]) cell.localBboxMin[a] = c->localBboxMin[a];
            if (c->localBboxMax[a] > cell.localBboxMax[a]) cell.localBboxMax[a] = c->localBboxMax[a];
        }
    }
}

// ===========================================================================
// TilesetWriter — tileset.json generation + tile writing
// ===========================================================================

int TilesetWriter::CountDescendantContent(const GridCell& cell)
{
    int n = cell.hasContent ? 1 : 0;
    for (auto& c : cell.children)
        if (c) n += CountDescendantContent(*c);
    return n;
}

void TilesetWriter::CollectContentSubtree(const GridCell& cell,
                                          std::vector<const GridCell*>& out)
{
    if (cell.hasContent) out.push_back(&cell);
    for (auto& c : cell.children)
        if (c) CollectContentSubtree(*c, out);
}

void TilesetWriter::CollectContentCells(GridCell& cell,
                                        std::vector<GridCell*>& out)
{
    if (cell.hasContent) out.push_back(&cell);
    for (auto& c : cell.children)
        if (c) CollectContentCells(*c, out);
}

double TilesetWriter::ComputeCellGeometricErrors(const GridCell& cell,
                                                  double parentGeomErr)
{
    double diag = BBoxUtils::Diagonal(cell.localBboxMin, cell.localBboxMax);
    double geomErr = diag * TileConstants::GEOMETRIC_ERROR_COEFFICIENT;
    if (geomErr > parentGeomErr * 0.9)
        geomErr = parentGeomErr * 0.85;
    if (geomErr < 1.0) geomErr = 1.0;

    double maxChildGeomErr = 0;
    for (auto& c : cell.children)
    {
        if (!c) continue;
        if (!c->hasContent && !c->hasChildren()) continue;
        double childGe = ComputeCellGeometricErrors(*c, geomErr);
        if (childGe > maxChildGeomErr) maxChildGeomErr = childGe;
    }

    if (maxChildGeomErr > geomErr)
        geomErr = maxChildGeomErr;

    cell.computedGeomError = geomErr;
    return geomErr;
}

nlohmann::ordered_json TilesetWriter::WriteNodeRecursive(const GridCell& cell,
                                                         double parentGeomErr,
                                                         const std::string& b3dmRelBase,
                                                         const std::string& refine)
{
    nlohmann::ordered_json node;
    node["boundingVolume"] = BBoxUtils::WriteBoxJson(cell.localBboxMin, cell.localBboxMax);

    double geomErr = cell.computedGeomError;
    if (geomErr <= 0.0)
    {
        double diag = BBoxUtils::Diagonal(cell.localBboxMin, cell.localBboxMax);
        geomErr = diag * TileConstants::GEOMETRIC_ERROR_COEFFICIENT;
        if (geomErr > parentGeomErr * 0.9)
            geomErr = parentGeomErr * 0.5;
        if (geomErr < 1.0) geomErr = 1.0;
    }
    node["geometricError"] = geomErr;
    node["refine"] = refine;

    if (cell.hasContent && !cell.tileFileName.empty())
    {
        std::string relUri = b3dmRelBase.empty()
            ? (levelDir(cell) + "/" + cell.tileFileName)
            : (b3dmRelBase + "/" + levelDir(cell) + "/" + cell.tileFileName);
        node["content"] = {{"uri", relUri}};
    }

    std::vector<const GridCell*> validChildren;
    for (auto& c : cell.children)
    {
        if (!c) continue;
        if (!c->hasContent && !c->hasChildren()) continue;
        validChildren.push_back(c.get());
    }

    if (!validChildren.empty())
    {
        nlohmann::ordered_json children = nlohmann::ordered_json::array();
        for (auto* c : validChildren)
            children.push_back(WriteNodeRecursive(*c, geomErr, b3dmRelBase, refine));
        node["children"] = children;
    }

    return node;
}

bool TilesetWriter::WriteSubtreeTileset(const GridCell& cell,
                                        const TileBuildOptions& opts,
                                        const std::string& subdir)
{
    std::vector<const GridCell*> contentCells;
    CollectContentSubtree(cell, contentCells);
    if (contentCells.empty()) return false;

    std::string dirPath = opts.outputDir + "/" + subdir;
    fs::create_directories(dirPath);
    std::string jsonPath = dirPath + "/tileset.json";

    std::string b3dmRelBase = "";

    double diag = BBoxUtils::Diagonal(cell.localBboxMin, cell.localBboxMax);
    double rootGeomErr = diag * TileConstants::GEOMETRIC_ERROR_COEFFICIENT;
    if (rootGeomErr < opts.minBlockDistance)
        rootGeomErr = opts.minBlockDistance;

    double computedRoot = ComputeCellGeometricErrors(cell, rootGeomErr * 2.0);

    if (computedRoot < rootGeomErr)
    {
        const_cast<GridCell&>(cell).computedGeomError = rootGeomErr;
        computedRoot = rootGeomErr;
    }

    nlohmann::ordered_json j;
    j["asset"] = {{"version", "1.1"}, {"generator", "MGO TileBuilder"}};
    j["geometricError"] = rootGeomErr;
    j["root"] = WriteNodeRecursive(cell, rootGeomErr * 2.0, b3dmRelBase, opts.refine);

    std::ofstream f(jsonPath);
    if (!f)
    {
        MGO_LOG(Error) << "[TileBuilder] Cannot write " << jsonPath;
        return false;
    }
    f << j.dump(2) << "\n";
    f.close();

    MGO_LOG(Info) << "[TileBuilder] Wrote " << jsonPath
              << " (" << contentCells.size() << " content cells in subtree)"
              ;
    return true;
}

bool TilesetWriter::WriteTiles(GridCell& cell, const aiScene* scene,
                               const TileBuildOptions& opts,
                               const std::string& subdir)
{
    // Depth-first: recurse into children first
    for (auto& c : cell.children)
        if (c && !WriteTiles(*c, scene, opts, subdir)) return false;

    if (!cell.hasContent || cell.instances.empty()) return true;

    MaterialGrouper::GroupCellByMaterial(cell, scene, opts);

    if (cell.materialGroups.empty()) return true;

    BinaryBlob glb;
    if (!GlbBuilder::Build(cell.materialGroups, glb, opts.fbxDirectory,
                           opts.doubleSided)) return false;

    BinaryBlob b3dm;
    if (!B3dmBuilder::Build(glb, b3dm)) return false;

    // Build file path FIRST, write file, then set tileFileName only on success.
    // Setting tileFileName before the write would leave a dangling content.uri
    // in tileset.json pointing to a non-existent .b3dm if write fails.
    std::string dirPath;
    std::string tileName;
    if (cell.isOverflow)
    {
        // Always place overflow b3dm under the "overflow" levelDir, so the
        // content.uri ("overflow/tile_overflow.b3dm") resolves relative to
        // BOTH the root tileset.json and an external subtree tileset.json.
        // Previously an external overflow subtree wrote to <out>/<subdir>/
        // while the subtree json referenced <out>/<subdir>/overflow/ -> 404.
        std::string prefix = subdir.empty() ? "" : (subdir + "/");
        dirPath = opts.outputDir + "/" + prefix + "overflow";
        tileName = "tile_overflow.b3dm";
    }
    else
    {
        std::string ld = "L" + std::to_string(cell.level);
        std::string prefix = subdir.empty() ? "" : (subdir + "/");
        dirPath = opts.outputDir + "/" + prefix + ld;
        tileName = opts.tileBaseName + "_" + cell.cellKey + ".b3dm";
    }
    fs::create_directories(dirPath);

    std::string tilePath = dirPath + "/" + tileName;
    std::ofstream f(tilePath, std::ios::binary);
    if (!f)
    {
        MGO_LOG(Error) << "[TileBuilder] Cannot write " << tilePath;
        return false;
    }
    cell.tileFileName = std::move(tileName);
    f.write(reinterpret_cast<const char*>(b3dm.ptr()),
            static_cast<std::streamsize>(b3dm.size()));
    f.close();

    static int s_written = 0;
    ++s_written;
    MGO_LOG(Info) << "[TileBuilder] [" << s_written << "] Wrote " << tilePath
              << " (" << b3dm.size() << " bytes, "
              << cell.materialGroups.size() << " material(s), "
              << cell.totalVertexCount() << " vertices)";
    return true;
}

bool TilesetWriter::Generate(const GridCell& root,
                             const TileBuildOptions& opts,
                             const Eigen::Matrix4d* rootTransform,
                             std::string& outJson)
{
    const int MIN_CONTENT_FOR_EXTERNAL = TileConstants::MIN_CONTENT_FOR_EXTERNAL_TILESET;

    // 1. Write external tileset files for subtrees laid out under a subdir.
    //    The external/inline decision is made by the converter (cell.isExternal)
    //    so it matches where the .b3dm files were actually written - NOT
    //    recomputed here, or a subtree whose tiny cells were dropped during
    //    grouping would flip to inline and strand its files (404).
    for (auto& c : root.children)
    {
        if (!c) continue;
        if (!c->isExternal) continue;
        if (CountDescendantContent(*c) == 0) continue;

        std::string subdir = c->isOverflow
            ? "overflow"
            : (opts.tileBaseName + "_" + c->cellKey);
        if (!WriteSubtreeTileset(*c, opts, subdir)) { MGO_LOG(Error) << "[TileBuilder] Failed to write external tileset for " << subdir; return false; }
    }

    // 2. Write root tileset.json
    std::string jsonPath = opts.outputDir + "/tileset.json";

    // Compute root local bbox from immediate non-empty children
    double rootLocalMin[3] = { DBL_MAX, DBL_MAX, DBL_MAX };
    double rootLocalMax[3] = { -DBL_MAX, -DBL_MAX, -DBL_MAX };
    bool hasValidChild = false;
    for (auto& c : root.children)
    {
        if (!c) continue;
        if (!c->hasContent && !c->hasChildren() && c->materialGroups.empty()) continue;
        if (c->localBboxMin[0] < -1e10 || c->localBboxMin[1] < -1e10 || c->localBboxMin[2] < -1e10 ||
            c->localBboxMax[0] > 1e10 || c->localBboxMax[1] > 1e10 || c->localBboxMax[2] > 1e10)
            continue;

        if (!hasValidChild)
        {
            for (int a = 0; a < 3; ++a)
            {
                rootLocalMin[a] = c->localBboxMin[a];
                rootLocalMax[a] = c->localBboxMax[a];
            }
            hasValidChild = true;
        }
        else
        {
            for (int a = 0; a < 3; ++a)
            {
                if (c->localBboxMin[a] < rootLocalMin[a]) rootLocalMin[a] = c->localBboxMin[a];
                if (c->localBboxMax[a] > rootLocalMax[a]) rootLocalMax[a] = c->localBboxMax[a];
            }
        }
    }
    double rootDiag = BBoxUtils::Diagonal(rootLocalMin, rootLocalMax);
    double rootError = opts.rootGeometricError > 0
        ? opts.rootGeometricError : rootDiag;
    double rootGeomErr = rootDiag * TileConstants::GEOMETRIC_ERROR_COEFFICIENT;
    if (rootGeomErr < opts.minBlockDistance * (1 << opts.maxLODLevels))
        rootGeomErr = opts.minBlockDistance * (1 << opts.maxLODLevels);

    // Spec: the tileset-level geometricError is the error when the whole
    // tileset is NOT rendered, so it must be >= the root tile's error.
    if (rootError < rootGeomErr)
        rootError = rootGeomErr;

    bool hasTransform = (rootTransform != nullptr);

    nlohmann::ordered_json j;
    j["asset"] = {{"version", "1.1"}, {"generator", "MGO TileBuilder"}};
    j["geometricError"] = rootError;
    j["root"] = nlohmann::ordered_json::object();
    j["root"]["boundingVolume"] = BBoxUtils::WriteBoxJson(rootLocalMin, rootLocalMax);
    j["root"]["geometricError"] = rootGeomErr;
    j["root"]["refine"] = opts.refine;
    if (hasTransform)
    {
        // Eigen column-major .data() order == the old column-major out[16]
        const double* t = rootTransform->data();
        nlohmann::ordered_json transform = nlohmann::ordered_json::array();
        for (int i = 0; i < 16; ++i)
            transform.push_back(t[i]);
        j["root"]["transform"] = transform;
    }

    // Build root's children using the converter's persisted external/inline
    // decision (same rationale as step 1: consistency with the file layout).
    std::vector<const GridCell*> externalKids;
    std::vector<const GridCell*> inlineKids;
    for (auto& c : root.children)
    {
        if (!c) continue;
        if (CountDescendantContent(*c) == 0) continue;
        if (c->isExternal)
            externalKids.push_back(c.get());
        else
            inlineKids.push_back(c.get());
    }

    bool hasChildren = !externalKids.empty() || !inlineKids.empty();
    if (hasChildren)
    {
        nlohmann::ordered_json children = nlohmann::ordered_json::array();

        for (auto* c : externalKids)
        {
            std::string uri = c->isOverflow
                ? "overflow/tileset.json"
                : (opts.tileBaseName + "_" + c->cellKey + "/tileset.json");
            double gErr = BBoxUtils::Diagonal(c->localBboxMin, c->localBboxMax) / 10.0;
            if (gErr < opts.minBlockDistance)
                gErr = opts.minBlockDistance;

            nlohmann::ordered_json n;
            n["boundingVolume"] = BBoxUtils::WriteBoxJson(c->localBboxMin, c->localBboxMax);
            n["geometricError"] = gErr;
            n["content"] = {{"uri", uri}};
            n["refine"] = opts.refine;
            children.push_back(n);
        }

        for (auto* c : inlineKids)
        {
            ComputeCellGeometricErrors(*c, rootGeomErr);
            children.push_back(WriteNodeRecursive(*c, rootGeomErr, "", opts.refine));
        }

        j["root"]["children"] = children;
    }

    outJson = j.dump(2) + "\n";

    std::ofstream f(jsonPath);
    if (!f)
    {
        MGO_LOG(Error) << "[TileBuilder] Cannot write " << jsonPath;
        return false;
    }
    f << outJson;
    f.close();

    MGO_LOG(Info) << "[TileBuilder] Wrote " << jsonPath;
    return true;
}
