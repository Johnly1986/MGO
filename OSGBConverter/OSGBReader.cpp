// Copyright Johnlyon
//
// OSGBReader implementation — OSGB → OSGBTileData via OpenSceneGraph
//
// Requires: libosg, libosgDB, libosgUtil
//   Linux:   apt install libopenscenegraph-dev
//   Windows: vcpkg install osg:x64-windows
//

#include "OSGBReader.h"
#include "MetadataParser.h"
#include "IVendorHandler.h"

// OSG headers
#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/NodeVisitor>
#include <osg/Array>
#include <osg/PrimitiveSet>
#include <osg/Texture2D>
#include <osg/Image>
#include <osg/Material>
#include <osg/StateSet>
#include <osg/GL>
#include <osgDB/ReadFile>

#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>
#include <algorithm>
#include <cmath>

// ---------------------------------------------------------------------------
// Internal helpers (free functions, no OSG headers in public API)
// ---------------------------------------------------------------------------

namespace {

// Extract triangle indices from a DrawElements primitive
void ExtractIndices(osg::PrimitiveSet* ps, std::vector<unsigned int>& outIndices,
                    unsigned int baseVertex)
{
    if (ps->getType() == osg::PrimitiveSet::DrawElementsUIntPrimitiveType ||
        ps->getType() == osg::PrimitiveSet::DrawElementsUShortPrimitiveType ||
        ps->getType() == osg::PrimitiveSet::DrawElementsUBytePrimitiveType)
    {
        unsigned int mode = ps->getMode();
        unsigned int numIndices = ps->getNumIndices();

        if (mode != osg::PrimitiveSet::TRIANGLES &&
            mode != osg::PrimitiveSet::TRIANGLE_STRIP &&
            mode != osg::PrimitiveSet::TRIANGLE_FAN)
        {
            return;
        }

        const osg::DrawElements* de = static_cast<const osg::DrawElements*>(ps);

        if (mode == osg::PrimitiveSet::TRIANGLES)
        {
            for (unsigned int i = 0; i < numIndices; ++i)
                outIndices.push_back(de->index(i) + baseVertex);
        }
        else if (mode == osg::PrimitiveSet::TRIANGLE_STRIP)
        {
            for (unsigned int i = 0; i + 2 < numIndices; ++i)
            {
                if (i % 2 == 0)
                {
                    outIndices.push_back(de->index(i)     + baseVertex);
                    outIndices.push_back(de->index(i + 1) + baseVertex);
                    outIndices.push_back(de->index(i + 2) + baseVertex);
                }
                else
                {
                    outIndices.push_back(de->index(i)     + baseVertex);
                    outIndices.push_back(de->index(i + 2) + baseVertex);
                    outIndices.push_back(de->index(i + 1) + baseVertex);
                }
            }
        }
        else if (mode == osg::PrimitiveSet::TRIANGLE_FAN)
        {
            for (unsigned int i = 1; i + 1 < numIndices; ++i)
            {
                outIndices.push_back(de->index(0)     + baseVertex);
                outIndices.push_back(de->index(i)     + baseVertex);
                outIndices.push_back(de->index(i + 1) + baseVertex);
            }
        }
    }
}

// Extract texture path from a StateSet
std::string ExtractTexturePath(osg::StateSet* ss)
{
    if (!ss) return "";

    const osg::StateSet::TextureAttributeList& texAttrs = ss->getTextureAttributeList();
    for (const auto& texAttrVec : texAttrs)
    {
        for (const auto& texAttr : texAttrVec)
        {
            osg::Texture2D* t = dynamic_cast<osg::Texture2D*>(texAttr.second.first.get());
            if (t && t->getImage())
                return t->getImage()->getFileName();
        }
    }
    return "";
}

// Extract the texture image from a StateSet (for embedded texture pixels).
osg::Image* ExtractTextureImage(osg::StateSet* ss)
{
    if (!ss) return nullptr;

    const osg::StateSet::TextureAttributeList& texAttrs = ss->getTextureAttributeList();
    for (const auto& texAttrVec : texAttrs)
    {
        for (const auto& texAttr : texAttrVec)
        {
            osg::Texture2D* t = dynamic_cast<osg::Texture2D*>(texAttr.second.first.get());
            if (t && t->getImage())
                return t->getImage();
        }
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// DXT (S3TC) decompression. ContextCapture / DJI Terra frequently embed
// DXT1/DXT3/DXT5 compressed textures inside OSGB; the compressed bytes must
// be decoded, otherwise they leak into the output as garbage pixels.
// Rows are emitted in the same order as the compressed blocks (memory order),
// preserving OSG's sampling semantics.
// ---------------------------------------------------------------------------

// Local copies of the GL enums (not all OSG builds export the S3TC defines).
constexpr uint32_t kDXT1RGB  = 0x83F0;  // GL_COMPRESSED_RGB_S3TC_DXT1_EXT
constexpr uint32_t kDXT1RGBA = 0x83F1;  // GL_COMPRESSED_RGBA_S3TC_DXT1_EXT
constexpr uint32_t kDXT3     = 0x83F2;  // GL_COMPRESSED_RGBA_S3TC_DXT3_EXT
constexpr uint32_t kDXT5     = 0x83F3;  // GL_COMPRESSED_RGBA_S3TC_DXT5_EXT

// Decode one 4x4 DXT color block into 16 RGBA pixels.
//   c: 8 bytes (c0, c1 as 565, then 4 bytes of 2-bit indices)
//   threeColorMode: DXT1 with c0 <= c1 uses a 3-color + transparent palette.
void DecodeDXTColorBlock(const uint8_t* c, bool threeColorMode, uint8_t out[16 * 4])
{
    auto expand565 = [](uint16_t v, uint8_t& r, uint8_t& g, uint8_t& b) {
        r = static_cast<uint8_t>(((v >> 11) & 0x1F) * 255 / 31);
        g = static_cast<uint8_t>(((v >> 5) & 0x3F) * 255 / 63);
        b = static_cast<uint8_t>((v & 0x1F) * 255 / 31);
    };

    uint16_t c0 = static_cast<uint16_t>(c[0] | (c[1] << 8));
    uint16_t c1 = static_cast<uint16_t>(c[2] | (c[3] << 8));
    uint8_t p[4][3];
    expand565(c0, p[0][0], p[0][1], p[0][2]);
    expand565(c1, p[1][0], p[1][1], p[1][2]);
    if (threeColorMode && c0 <= c1)
    {
        for (int i = 0; i < 3; ++i)
        {
            p[2][i] = static_cast<uint8_t>((p[0][i] + p[1][i]) / 2);
        }
        p[3][0] = p[3][1] = p[3][2] = 0;  // transparent black
    }
    else
    {
        for (int i = 0; i < 3; ++i)
        {
            p[2][i] = static_cast<uint8_t>((2 * p[0][i] + p[1][i]) / 3);
            p[3][i] = static_cast<uint8_t>((p[0][i] + 2 * p[1][i]) / 3);
        }
    }

    uint32_t idx = static_cast<uint32_t>(c[4]) | (c[5] << 8) | (c[6] << 16) | (c[7] << 24);
    for (int px = 0; px < 16; ++px)
    {
        int sel = (idx >> (2 * px)) & 3;
        bool transparent = threeColorMode && c0 <= c1 && sel == 3;
        out[px * 4 + 0] = p[sel][0];
        out[px * 4 + 1] = p[sel][1];
        out[px * 4 + 2] = p[sel][2];
        out[px * 4 + 3] = transparent ? 0 : 255;
    }
}

bool DecodeDXT(const uint8_t* src, size_t srcSize, uint32_t format,
               int w, int h, std::vector<uint8_t>& rgba)
{
    if (w <= 0 || h <= 0) return false;

    const bool isDXT1 = (format == kDXT1RGB || format == kDXT1RGBA);
    const size_t blockSize = isDXT1 ? 8 : 16;
    const int blocksX = (w + 3) / 4;
    const int blocksY = (h + 3) / 4;
    if (srcSize < static_cast<size_t>(blocksX) * blocksY * blockSize)
        return false;

    rgba.assign(static_cast<size_t>(w) * h * 4, 255);

    for (int by = 0; by < blocksY; ++by)
    {
        for (int bx = 0; bx < blocksX; ++bx)
        {
            const uint8_t* blk = src + (static_cast<size_t>(by) * blocksX + bx) * blockSize;
            const uint8_t* colorBlock = isDXT1 ? blk : blk + 8;

            // Per-pixel alpha for DXT3/DXT5
            uint8_t alpha[16];
            if (format == kDXT3)
            {
                for (int i = 0; i < 16; i += 2)
                {
                    uint8_t b = blk[i / 2];
                    alpha[i]     = static_cast<uint8_t>((b & 0x0F) * 17);
                    alpha[i + 1] = static_cast<uint8_t>((b >> 4) * 17);
                }
            }
            else if (format == kDXT5)
            {
                uint8_t a0 = blk[0], a1 = blk[1];
                uint8_t pal[8];
                pal[0] = a0;
                pal[1] = a1;
                if (a0 > a1)
                {
                    for (int i = 2; i <= 7; ++i)
                        pal[i] = static_cast<uint8_t>(((8 - i) * a0 + (i - 1) * a1) / 7);
                }
                else
                {
                    for (int i = 2; i <= 5; ++i)
                        pal[i] = static_cast<uint8_t>(((6 - i) * a0 + (i - 1) * a1) / 5);
                    pal[6] = 0;
                    pal[7] = 255;
                }
                // 16 3-bit indices packed LSB-first into 6 bytes
                uint64_t bits = 0;
                for (int i = 0; i < 6; ++i)
                    bits |= static_cast<uint64_t>(blk[2 + i]) << (8 * i);
                for (int i = 0; i < 16; ++i)
                    alpha[i] = pal[(bits >> (3 * i)) & 7];
            }

            uint8_t px[16 * 4];
            DecodeDXTColorBlock(colorBlock, format != kDXT1RGB, px);

            for (int y = 0; y < 4; ++y)
            {
                int dy = by * 4 + y;
                if (dy >= h) break;
                for (int x = 0; x < 4; ++x)
                {
                    int dx = bx * 4 + x;
                    if (dx >= w) break;
                    int srcPx = y * 4 + x;
                    size_t dstIdx = (static_cast<size_t>(dy) * w + dx) * 4;
                    rgba[dstIdx + 0] = px[srcPx * 4 + 0];
                    rgba[dstIdx + 1] = px[srcPx * 4 + 1];
                    rgba[dstIdx + 2] = px[srcPx * 4 + 2];
                    rgba[dstIdx + 3] =
                        (format == kDXT3 || format == kDXT5) ? alpha[srcPx] : px[srcPx * 4 + 3];
                }
            }
        }
    }
    return true;
}

// Convert a decoded osg::Image to tightly-packed RGBA8. Returns false if the
// image has no usable pixel data (unsupported compressed format, or unknown
// layout). Output rows preserve the image's memory row order.
bool ImageToRGBA(osg::Image* img, std::vector<uint8_t>& rgba, int& w, int& h)
{
    rgba.clear();
    w = 0;
    h = 0;
    if (!img || !img->data()) return false;

    w = img->s();
    h = img->t();

    // Compressed textures: decode DXT variants, reject the rest.
    if (img->isCompressed())
    {
        uint32_t fmt = img->getPixelFormat();
        const uint8_t* src = reinterpret_cast<const uint8_t*>(img->data());
        bool ok = false;
        if (fmt == kDXT1RGB || fmt == kDXT1RGBA || fmt == kDXT3 || fmt == kDXT5)
            ok = DecodeDXT(src, img->getTotalSizeInBytes(), fmt, w, h, rgba);
        if (!ok)
        {
            std::cerr << "[OSGBReader] Unsupported compressed texture format 0x"
                      << std::hex << fmt << std::dec
                      << " (" << w << "x" << h << ")" << std::endl;
            w = 0;
            h = 0;
        }
        return ok;
    }

    // Uncompressed: require 8-bit components.
    if (img->getDataType() != GL_UNSIGNED_BYTE)
    {
        std::cerr << "[OSGBReader] Unsupported texture data type 0x"
                  << std::hex << img->getDataType() << std::dec << std::endl;
        return false;
    }

    const uint32_t fmt = img->getPixelFormat();
    const uint8_t* src = img->data();
    size_t n = static_cast<size_t>(w) * h;

    bool bgr = (fmt == GL_BGR || fmt == GL_BGRA);
    int channels;
    switch (fmt)
    {
        case GL_RGB:  channels = 3; break;
        case GL_BGR:  channels = 3; break;
        case GL_RGBA: channels = 4; break;
        case GL_BGRA: channels = 4; break;
        case GL_LUMINANCE:          channels = 1; break;
        case GL_LUMINANCE_ALPHA:    channels = 2; break;
        default:
            std::cerr << "[OSGBReader] Unsupported texture pixel format 0x"
                      << std::hex << fmt << std::dec << std::endl;
            return false;
    }

    if (channels >= 3)
    {
        int c = channels;
        rgba.resize(n * 4);
        for (size_t i = 0; i < n; ++i)
        {
            const uint8_t* p = src + i * c;
            rgba[i * 4 + 0] = bgr ? p[2] : p[0];
            rgba[i * 4 + 1] = p[1];
            rgba[i * 4 + 2] = bgr ? p[0] : p[2];
            rgba[i * 4 + 3] = (c == 4) ? p[3] : 255;
        }
    }
    else if (channels == 2)  // luminance + alpha
    {
        rgba.resize(n * 4);
        for (size_t i = 0; i < n; ++i)
        {
            rgba[i * 4 + 0] = src[i * 2 + 0];
            rgba[i * 4 + 1] = src[i * 2 + 0];
            rgba[i * 4 + 2] = src[i * 2 + 0];
            rgba[i * 4 + 3] = src[i * 2 + 1];
        }
    }
    else  // luminance
    {
        rgba.resize(n * 4);
        for (size_t i = 0; i < n; ++i)
        {
            rgba[i * 4 + 0] = src[i];
            rgba[i * 4 + 1] = src[i];
            rgba[i * 4 + 2] = src[i];
            rgba[i * 4 + 3] = 255;
        }
    }
    return true;
}

// Extract material properties from a StateSet
void ExtractMaterial(osg::StateSet* ss, float baseColorFactor[4])
{
    if (!ss) return;

    osg::StateAttribute* attr = ss->getAttribute(osg::StateAttribute::MATERIAL);
    osg::Material* mat = dynamic_cast<osg::Material*>(attr);
    if (mat)
    {
        osg::Vec4 diffuse = mat->getDiffuse(osg::Material::FRONT);
        baseColorFactor[0] = diffuse.r();
        baseColorFactor[1] = diffuse.g();
        baseColorFactor[2] = diffuse.b();
        baseColorFactor[3] = diffuse.a();
    }
}

// ---------------------------------------------------------------------------
// GeometryCollector: OSG scene graph visitor that extracts geometry
// ---------------------------------------------------------------------------

class GeometryCollector : public osg::NodeVisitor
{
public:
    GeometryCollector(OSGBTileData& data, const std::string& tileDir,
                      const std::string& rootDir)
        : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
        , m_data(data)
        , m_tileDir(tileDir)
        , m_rootDir(rootDir)
    {
    }

    virtual void apply(osg::MatrixTransform& node) override
    {
        osg::Matrix prevMatrix = m_currentMatrix;
        m_currentMatrix = m_currentMatrix * node.getMatrix();

        std::string prevTexture = m_currentTexture;
        osg::ref_ptr<osg::Image> prevImage = m_currentImage;
        osg::StateSet* ss = node.getStateSet();
        if (ss)
        {
            std::string t = ExtractTexturePath(ss);
            if (!t.empty()) { m_currentTexture = ResolveTexturePath(t); m_currentImage = ExtractTextureImage(ss); }
            ExtractMaterial(ss, m_data.baseColorFactor);
        }

        traverse(node);
        m_currentMatrix = prevMatrix;
        m_currentTexture = prevTexture;
        m_currentImage = prevImage;
    }

    virtual void apply(osg::Geode& node) override
    {
        std::string prevTexture = m_currentTexture;
        osg::ref_ptr<osg::Image> prevImage = m_currentImage;
        osg::StateSet* ss = node.getStateSet();
        if (ss)
        {
            std::string t = ExtractTexturePath(ss);
            if (!t.empty()) { m_currentTexture = ResolveTexturePath(t); m_currentImage = ExtractTextureImage(ss); }
            ExtractMaterial(ss, m_data.baseColorFactor);
        }

        for (unsigned int i = 0; i < node.getNumDrawables(); ++i)
        {
            osg::Drawable* drawable = node.getDrawable(i);
            osg::Geometry* geom = drawable->asGeometry();
            if (geom)
                ProcessGeometry(geom);
        }

        traverse(node);
        m_currentTexture = prevTexture;
        m_currentImage = prevImage;
    }

private:
    // Prefix a texture filename with the tile's directory so GlbBuilder can
    // resolve it against the input root. ContextCapture stores textures next to
    // the OSGB tiles, but osg::Image::getFileName() returns only the basename,
    // so without this the file lookup happens in the wrong directory and the
    // texture is silently dropped (empty MIME type in the glTF).
    std::string ResolveTexturePath(const std::string& texPath) const
    {
        if (texPath.empty() || m_tileDir.empty()) return texPath;
        if (texPath.size() >= 2 && texPath[1] == ':') return texPath;  // drive
        if (texPath[0] == '/' || texPath[0] == '\\') return texPath;   // absolute
        if (texPath.size() > m_tileDir.size() &&
            texPath.compare(0, m_tileDir.size(), m_tileDir) == 0 &&
            (texPath[m_tileDir.size()] == '/' || texPath[m_tileDir.size()] == '\\'))
            return texPath;  // already prefixed
        return m_tileDir + "/" + texPath;
    }

    // Find (or create) the per-texture group that this geometry belongs to.
    // Returns an index so callers can safely re-reference m_data.groups (which
    // may reallocate on push_back).
    size_t GetOrCreateGroup(const std::string& texturePath)
    {
        for (size_t i = 0; i < m_data.groups.size(); ++i)
            if (m_data.groups[i].texturePath == texturePath)
                return i;

        TextureGroup g;
        g.texturePath = texturePath;
        for (int i = 0; i < 4; ++i)
            g.baseColorFactor[i] = m_data.baseColorFactor[i];
        m_data.groups.push_back(std::move(g));
        return m_data.groups.size() - 1;
    }

    void ProcessGeometry(osg::Geometry* geom)
    {
        // Resolve the texture for this geometry: the geometry's own StateSet
        // wins, otherwise inherit from ancestor nodes (m_currentTexture).
        std::string tex = m_currentTexture;
        osg::Image* texImage = m_currentImage.get();
        if (geom->getStateSet())
        {
            std::string t = ExtractTexturePath(geom->getStateSet());
            if (!t.empty()) { tex = ResolveTexturePath(t); texImage = ExtractTextureImage(geom->getStateSet()); }
            ExtractMaterial(geom->getStateSet(), m_data.baseColorFactor);
        }

        osg::Array* va = geom->getVertexArray();
        if (!va) return;

        osg::Vec3Array* vertices = dynamic_cast<osg::Vec3Array*>(va);
        if (!vertices) return;

        unsigned int numVertices = vertices->size();
        // Group key: texture path PLUS the embedded image object identity.
        // OSGB embedded textures carry virtual filenames that may be empty or
        // identical across different images; keying by path alone merges
        // geometry of different textures into one group whose pixels come
        // from only the FIRST image - every other texture's geometry then
        // renders with a wrong texture.
        std::string texKey = tex;
        if (texImage)
            texKey += "#" + std::to_string(
                reinterpret_cast<unsigned long long>(texImage));
        size_t gi = GetOrCreateGroup(texKey);
        TextureGroup& grp = m_data.groups[gi];
        unsigned int baseVertex = static_cast<unsigned int>(grp.VertexCount());

        // Capture texture pixels once per unique texture group. Embedded
        // images are the primary source (ContextCapture embeds textures with
        // virtual filenames). When the texture is only referenced as an
        // external file, load it through osgDB so the embedded pixels follow
        // the exact same memory-row semantics the model's UVs were authored
        // against - a raw file embed would bypass that and break alignment.
        if (grp.texturePixels.empty())
        {
            osg::Image* img = texImage;
            osg::ref_ptr<osg::Image> fileImg;
            if (!img && !tex.empty())
            {
                auto it = m_fileImageCache.find(tex);
                if (it != m_fileImageCache.end())
                {
                    fileImg = it->second;
                }
                else
                {
                    std::string full = m_rootDir + "/" + tex;
                    fileImg = osgDB::readImageFile(full);
                    m_fileImageCache[tex] = fileImg;  // null = negative cache
                }
                img = fileImg.get();
            }
            if (img)
                ImageToRGBA(img, grp.texturePixels, grp.textureWidth, grp.textureHeight);
        }

        // Transform and accumulate vertex positions
        for (unsigned int i = 0; i < numVertices; ++i)
        {
            osg::Vec3 v = vertices->at(i) * m_currentMatrix;
            grp.positions.push_back(v.x());
            grp.positions.push_back(v.y());
            grp.positions.push_back(v.z());
        }

        // Normals (optional — left empty when absent; the cell builder then
        // computes smooth normals. Zero-padding violates the glTF spec, which
        // requires unit-length normals).
        osg::Array* na = geom->getNormalArray();
        osg::Vec3Array* normals = nullptr;
        if (na) normals = dynamic_cast<osg::Vec3Array*>(na);

        if (normals && normals->size() == numVertices)
        {
            osg::Matrixd normalMatrix;
            normalMatrix.invert(m_currentMatrix);
            normalMatrix.transpose(normalMatrix);

            for (unsigned int i = 0; i < numVertices; ++i)
            {
                osg::Vec3f n = osg::Matrix::transform3x3(normals->at(i), normalMatrix);
                n.normalize();
                grp.normals.push_back(n.x());
                grp.normals.push_back(n.y());
                grp.normals.push_back(n.z());
            }
        }
        // (absent normals stay empty; the cell builder computes them)

        // Texture coordinates (optional — pad with zeros, same reason as above)
        osg::Array* tca = geom->getTexCoordArray(0);
        osg::Vec2Array* texcoords = nullptr;
        if (tca) texcoords = dynamic_cast<osg::Vec2Array*>(tca);

        if (texcoords && texcoords->size() == numVertices)
        {
            for (unsigned int i = 0; i < numVertices; ++i)
            {
                osg::Vec2 uv = texcoords->at(i);
                // UVs pass through unchanged: OSG hands image memory rows to
                // OpenGL as-is (v=0 samples memory row 0; Texture.cpp never
                // flips for Image::Origin), and glTF samples stored row 0 at
                // uv.y=0. Identical bytes + identical UVs = identical sampling.
                grp.texcoords.push_back(uv.x());
                grp.texcoords.push_back(uv.y());
            }
        }
        else
        {
            grp.texcoords.insert(grp.texcoords.end(), numVertices * 2, 0.0f);
        }

        // Indices, offset by this group's current vertex count so each
        // per-texture group keeps a self-contained index buffer.
        for (unsigned int i = 0; i < geom->getNumPrimitiveSets(); ++i)
        {
            osg::PrimitiveSet* ps = geom->getPrimitiveSet(i);
            ExtractIndices(ps, grp.indices, baseVertex);
        }
    }

    OSGBTileData& m_data;
    std::string   m_tileDir;
    std::string   m_rootDir;
    std::map<std::string, osg::ref_ptr<osg::Image>> m_fileImageCache;
    osg::Matrixd  m_currentMatrix;
    std::string   m_currentTexture;
    osg::ref_ptr<osg::Image> m_currentImage;
};

} // anonymous namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool OSGBReader::ReadTile(const std::string& tilePath, const std::string& rootDir,
                           OSGBTileData& outData)
{
    std::string fullPath = rootDir + "/" + tilePath;

    osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(fullPath);
    if (!node)
    {
        m_lastError = "Failed to read OSGB file: " + fullPath;
        std::cerr << "[OSGBReader] " << m_lastError << std::endl;
        return false;
    }

    outData = OSGBTileData();
    outData.tilePath = tilePath;

    // Determine tile directory for texture resolution
    size_t pos = tilePath.find_last_of("/\\");
    std::string tileDir = (pos != std::string::npos) ? tilePath.substr(0, pos) : "";

    // Traverse the scene graph and collect geometry (per-texture groups)
    GeometryCollector collector(outData, tileDir, rootDir);
    node->accept(collector);

    // Compute per-group bounding boxes and the overall tile bounding box.
    outData.bboxMin[0] = outData.bboxMin[1] = outData.bboxMin[2] = 1e30;
    outData.bboxMax[0] = outData.bboxMax[1] = outData.bboxMax[2] = -1e30;
    for (auto& g : outData.groups)
    {
        g.ComputeBBox();
        for (int i = 0; i < 3; ++i)
        {
            if (g.bboxMin[i] < outData.bboxMin[i]) outData.bboxMin[i] = g.bboxMin[i];
            if (g.bboxMax[i] > outData.bboxMax[i]) outData.bboxMax[i] = g.bboxMax[i];
        }
    }
    if (outData.groups.empty())
        outData.ComputeBBox();  // legacy fallback (no groups produced)

    if (outData.IsEmpty())
    {
        m_lastError = "No triangle geometry found in: " + tilePath;
        return false;
    }

    return true;
}

int OSGBReader::ReadAllTiles(const std::vector<std::string>& tilePaths,
                              const std::string& rootDir,
                              IVendorHandler& handler,
                              int maxLOD,
                              std::vector<OSGBTileData>& outTiles)
{
    outTiles.clear();
    outTiles.reserve(tilePaths.size());

    int successCount = 0;
    int skippedCount = 0;
    for (const auto& path : tilePaths)
    {
        // Parse LOD/grid metadata first (cheap) so tiles above maxLOD can be
        // filtered out before the expensive OSG read.
        int lodLevel = 0, gridX = 0, gridY = 0;
        std::string subTileIndex;
        handler.ParseTilePath(path, lodLevel, gridX, gridY, subTileIndex);

        if (maxLOD > 0 && lodLevel > maxLOD)
        {
            ++skippedCount;
            continue;
        }

        OSGBTileData data;
        if (ReadTile(path, rootDir, data))
        {
            data.lodLevel = lodLevel;
            data.tileX = gridX;
            data.tileY = gridY;
            data.subTileIndex = subTileIndex;
            outTiles.push_back(std::move(data));
            ++successCount;
        }
    }

    if (skippedCount > 0)
        std::cerr << "[OSGBReader] 警告: 跳过 " << skippedCount
                  << " 个超过 maxLOD(" << maxLOD << ") 的瓦片" << std::endl;
    std::cout << "[OSGBReader] 处理进度: " << successCount << "/"
              << tilePaths.size() << std::endl;
    return successCount;
}