// Copyright Johnlyon
//

#include "MeshGroupOptimizer.h"
#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>

#include <algorithm>
#include <filesystem>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/ProgressHandler.hpp>
#include "meshoptimizer/meshoptimizer.h"
#include <boost/regex.hpp>
#include "../MeshProjectionErrorCorrector/Log.hpp"
#include "../MeshProjectionErrorCorrector/Error.hpp"

CMeshGroupOptimizer::CMeshGroupOptimizer()
{
	m_scene = nullptr;
	importer = nullptr;
}

CMeshGroupOptimizer::~CMeshGroupOptimizer()
{
	if(importer) delete importer;
}

std::string GetFileExtension(const std::string& filename)
{
	std::string extension = "obj";
	std::string::size_type pos = filename.find_last_of('.');
	if (pos != std::string::npos) extension = filename.substr(pos + 1);
	std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
	return extension;
}

const aiScene* CMeshGroupOptimizer::Load(const std::string& filename, bool rebuild)
{
	if (importer) { delete importer; importer = nullptr; m_scene = nullptr; }
	m_inputFilePath = filename;
	importer = new Assimp::Importer();
	std::string extension = GetFileExtension(filename);
	unsigned int options = rebuild ? aiProcessPreset_TargetRealtime_MaxQuality : aiProcess_GenSmoothNormals | aiProcess_Triangulate;
	options |= aiProcess_GenBoundingBoxes;
    //if(extension == "3ds") options |= aiProcess_ConvertToLeftHanded;

	m_scene = importer->ReadFile(filename, options);
	if (m_scene && m_scene->HasMeshes())
        return m_scene;
	if(importer->GetErrorString())
		throw MGO::Error(MGO::ErrorCode::FileReadError,
			    std::string("Assimp: ") + importer->GetErrorString());
	return nullptr;
}

bool CMeshGroupOptimizer::Optimize(const OptimizerConfig& config)
{
	if (!m_scene) return false;
	int totalorgiFaceNum = 0, totalOrgiVertexNum = 0, totalNewFaceNum = 0, totalNewVertexNum = 0;
	for (unsigned int mi = 0; mi < m_scene->mNumMeshes; mi++)
    {
        OptimizerItem optimizerItem;
		aiMesh* sectionMesh = m_scene->mMeshes[mi];
		if(!GetMatchedOptimizerItem(sectionMesh->mName.C_Str(), config, optimizerItem))
            continue;

		int orgiFaceNum, orgiVertexNum, newFaceNum, newVertexNum;

        orgiFaceNum = sectionMesh->mNumFaces;
        orgiVertexNum = sectionMesh->mNumVertices;

		bool isOptimization = PerformOptimization(sectionMesh, optimizerItem, config.reorder);

        newFaceNum = sectionMesh->mNumFaces;
        newVertexNum = sectionMesh->mNumVertices;

		if (!isOptimization)
		{
			MGO_LOG(Warning) << "mesh " << sectionMesh->mName.C_Str() << "Unable to complete optimization, skipping node";
            continue;
		}

		MGO_LOG(Info) << sectionMesh->mName.C_Str() <<
			" : Vertex simplification rate " << float(newVertexNum) / orgiVertexNum * 100 << "%" <<
			", Triangulation simplification rate " << float(newFaceNum) / orgiFaceNum * 100 << "%" <<
			std::endl;

		totalorgiFaceNum += orgiFaceNum;
        totalOrgiVertexNum += orgiVertexNum;
        totalNewFaceNum += newFaceNum;
        totalNewVertexNum += newVertexNum;
	}

	float tvr = totalOrgiVertexNum > 0 ? float(totalNewVertexNum) / totalOrgiVertexNum * 100 : 0;
    float ttr = totalorgiFaceNum > 0 ? float(totalNewFaceNum) / totalorgiFaceNum * 100 : 0;
	MGO_LOG(Info) << "Total Vertex simplification rate " << tvr << "%" <<
		", Total Triangulation simplification rate " << ttr << "%" <<
		std::endl;

	aiMatrix4x4 transformation(
		aiVector3D(1.f + config.scale),
		aiQuaternion(config.rotation.y, config.rotation.z, config.rotation.x),
		aiVector3D(config.move.x, config.move.y, config.move.z)
	);

	m_scene->mRootNode->mTransformation = m_scene->mRootNode->mTransformation * transformation;

	return true;
}

bool CMeshGroupOptimizer::Save(const std::string& filename, bool isConvertLeftHand)
{
	if (!m_scene) return false;
	Assimp::Exporter exporter;
	std::string extension = GetFileExtension(filename);

	// assimp registers its legacy glTF 1.0 exporters as "glb"/"gltf"; the 2.0
	// ones are "glb2"/"gltf2".  glTF 1.0 is not readable by Cesium (>=1.100 is
	// 2.0-only) nor by current tooling: loading such a .glb in the viewer dies
	// with "Failed to load model ... reading 'buffer'".  Route the two
	// extensions to the 2.0 exporters (the file name/extension on disk is
	// unchanged — assimp takes the exporter by id, then writes to `filename`).
	if (extension == "glb") extension = "glb2";
	else if (extension == "gltf") extension = "gltf2";
	if (extension == "glb2" || extension == "gltf2")
		RelocateExternalTextures(filename);

	unsigned int options =
		aiProcess_JoinIdenticalVertices |
		aiProcess_Triangulate;
	if (isConvertLeftHand) options |= aiProcess_ConvertToLeftHanded;

	return exporter.Export(m_scene, extension, filename, options) == AI_SUCCESS;
}

bool CMeshGroupOptimizer::RelocateExternalTextures(const std::string& outFile)
{
	if (!m_scene || !m_scene->HasMaterials()) return false;

	namespace fs = std::filesystem;
	std::error_code ec;

	const fs::path outDir = fs::path(outFile).parent_path();
	const fs::path modelDir = fs::path(m_inputFilePath).parent_path();

	for (unsigned int mi = 0; mi < m_scene->mNumMaterials; ++mi)
	{
		aiMaterial* material = m_scene->mMaterials[mi];
		if (!material) continue;

		for (int type = aiTextureType_DIFFUSE; type <= AI_TEXTURE_TYPE_MAX; ++type)
		{
			const aiTextureType ttype = static_cast<aiTextureType>(type);
			const unsigned int count = material->GetTextureCount(ttype);
			for (unsigned int ti = 0; ti < count; ++ti)
			{
				aiString texPath;
				if (material->GetTexture(ttype, ti, &texPath) != AI_SUCCESS) continue;

				const std::string raw = texPath.C_Str();
				if (raw.empty() || raw[0] == '*') continue;   // 已内嵌或未设置

				// Windows 导出的模型常把贴图写成 `E:\dir\tex.png`，而 POSIX 的
				// path::filename() 不认反斜杠（会原样返回整串），先统一分隔符
				std::string norm = raw;
				std::replace(norm.begin(), norm.end(), '\\', '/');

				const std::string base = fs::path(norm).filename().string();
				if (base.empty()) continue;

				// 先在原路径找，再退回模型所在目录（FBX 常把贴图目录存成绝对路径）
				fs::path src;
				if (fs::exists(norm, ec)) src = norm;
				else {
					fs::path candidate = modelDir / base;
					if (fs::exists(candidate, ec)) src = candidate;
				}

				if (!src.empty() && !outDir.empty())
				{
					const fs::path dst = outDir / base;
					if (src != dst) fs::copy_file(src, dst, fs::copy_options::overwrite_existing, ec);
				}

				// 无论是否复制成功都改写为相对文件名：绝对路径（尤其 Windows 盘符）
				// 在浏览器里是无效 URL，只会产生 CORS/加载失败噪声
				if (raw != base)
				{
					aiString rel(base.c_str());
					material->RemoveProperty(AI_MATKEY_TEXTURE(ttype, ti));
					material->AddProperty(&rel, AI_MATKEY_TEXTURE(ttype, ti));
					aiString chk; material->GetTexture(ttype, ti, &chk);
				}
			}
		}
	}
	return true;
}

bool CMeshGroupOptimizer::SimplifyScene(const aiScene* scene, const OptimizerConfig& config)
{
    if (!scene || !scene->HasMeshes()) return false;

    int totalorgiFaceNum = 0, totalOrgiVertexNum = 0, totalNewFaceNum = 0, totalNewVertexNum = 0;
    for (unsigned int mi = 0; mi < scene->mNumMeshes; mi++)
    {
        OptimizerItem optimizerItem;
        aiMesh* sectionMesh = scene->mMeshes[mi];
        if (!GetMatchedOptimizerItem(sectionMesh->mName.C_Str(), config, optimizerItem))
            continue;

        int orgiFaceNum = sectionMesh->mNumFaces;
        int orgiVertexNum = sectionMesh->mNumVertices;

        bool isOptimization = PerformOptimization(sectionMesh, optimizerItem, config.reorder);

        int newFaceNum = sectionMesh->mNumFaces;
        int newVertexNum = sectionMesh->mNumVertices;

        if (!isOptimization)
        {
            MGO_LOG(Info) << "mesh " << sectionMesh->mName.C_Str() << " Unable to complete optimization, skipping node" ;
            continue;
        }

        MGO_LOG(Info) << sectionMesh->mName.C_Str() <<
            " : Vertex simplification rate " << float(newVertexNum) / orgiVertexNum * 100 << "%" <<
            ", Triangulation simplification rate " << float(newFaceNum) / orgiFaceNum * 100 << "%" <<
            std::endl;

        totalorgiFaceNum += orgiFaceNum;
        totalOrgiVertexNum += orgiVertexNum;
        totalNewFaceNum += newFaceNum;
        totalNewVertexNum += newVertexNum;
    }

    float tvr = totalOrgiVertexNum > 0 ? float(totalNewVertexNum) / totalOrgiVertexNum * 100 : 0;
    float ttr = totalorgiFaceNum > 0 ? float(totalNewFaceNum) / totalorgiFaceNum * 100 : 0;
    MGO_LOG(Info) << "Total Vertex simplification rate " << tvr << "%" <<
        ", Total Triangulation simplification rate " << ttr << "%" <<
        std::endl;

    return true;
}

bool CMeshGroupOptimizer::GetMatchedOptimizerItem(const std::string& name, const OptimizerConfig& config, OptimizerItem& optimizerItem)
{
    for (const OptimizerItem& item : config.items)
    {
		boost::regex pattern(item.name, boost::regex::icase | boost::regex::perl);

		if (boost::regex_match(name, pattern))
        {
			optimizerItem = item;
            return true;
        }
    }
    return false;
}

unsigned int CMeshGroupOptimizer::GetSimplificationOptions(const OptimizerItem& optimizerItem)
{
	unsigned int options = 0;
	if(optimizerItem.lockBorder) options |= meshopt_SimplifyLockBorder;
	if(optimizerItem.localError) options |= meshopt_SimplifyErrorAbsolute;

return options;
}

size_t CMeshGroupOptimizer::MergeVertices(std::vector<unsigned int>& indices, std::vector<Vector3>& vertices, std::vector<Vector3>& normals, std::vector<Vector2>& uvs)
{
	std::vector<unsigned int> remap(indices.size());
	size_t vertex_size = sizeof(Vector3);
	size_t uv_size = sizeof(Vector2);
	size_t normal_size = sizeof(Vector3);
	size_t orgi_vertex_count = vertices.size();

	if (indices.size() == 0 || vertices.size() == 0) return 0;

	meshopt_Stream streams[] = {
		{vertices.data(), vertex_size, vertex_size},
		{uvs.data(), uv_size, uv_size},
	};

	size_t resizeVertexNum = meshopt_generateVertexRemapMulti(remap.data(), indices.data(), indices.size(), vertices.size(), streams, 2);
	meshopt_remapIndexBuffer(indices.data(), indices.data(), indices.size(), remap.data());

	meshopt_remapVertexBuffer(vertices.data(), vertices.data(), vertices.size(), vertex_size, remap.data());
	vertices.resize(resizeVertexNum);

	if (orgi_vertex_count == normals.size())
	{
		meshopt_remapVertexBuffer(normals.data(), normals.data(), normals.size(), normal_size, remap.data());
		normals.resize(resizeVertexNum);
	}

	if (orgi_vertex_count == uvs.size())
	{
		meshopt_remapVertexBuffer(uvs.data(), uvs.data(), uvs.size(), uv_size, remap.data());
		uvs.resize(resizeVertexNum);
	}
	return resizeVertexNum;
}

size_t CMeshGroupOptimizer::KeepEffectiveVertices(std::vector<unsigned int>& indices, std::vector<Vector3>& vertices, std::vector<Vector3>& normals, std::vector<Vector2>& uvs)
{
	std::vector<unsigned int> remap(indices.size());
	size_t vertex_size = sizeof(Vector3);
	size_t orgi_vertex_count = vertices.size();

	if (indices.size() == 0 || orgi_vertex_count == 0)
		return 0;

	size_t optimizeVertexNum = meshopt_optimizeVertexFetchRemap(remap.data(), indices.data(), indices.size(), vertices.size());
	meshopt_remapIndexBuffer(indices.data(), indices.data(), indices.size(), remap.data());
	meshopt_remapVertexBuffer(vertices.data(), vertices.data(), vertices.size(), vertex_size, remap.data());
	vertices.resize(optimizeVertexNum);

	if (orgi_vertex_count == normals.size())
	{
		size_t normal_size = sizeof(Vector3);
		meshopt_remapVertexBuffer(normals.data(), normals.data(), normals.size(), normal_size, remap.data());
		normals.resize(optimizeVertexNum);
	}

	if (orgi_vertex_count == uvs.size())
	{
		size_t uv_size = sizeof(Vector2);
		meshopt_remapVertexBuffer(uvs.data(), uvs.data(), uvs.size(), uv_size, remap.data());
		uvs.resize(optimizeVertexNum);
	}

	return optimizeVertexNum;
}

bool CMeshGroupOptimizer::PerformOptimization(aiMesh* mesh, const OptimizerItem& optimizerItem, bool reorder)
{
	size_t vertex_size = sizeof(Vector3);
	size_t normal_size = sizeof(Vector3);
	size_t uv_size = sizeof(Vector2);
	unsigned int orgiFaceNum = mesh->mNumFaces;
	unsigned int orgiVertexNum = mesh->mNumVertices;

	if(optimizerItem.error == 0)
        return true;

	if (mesh->HasFaces() == false || mesh->HasPositions() == false)
	{
		return false;
	}

	std::vector<Vector3> vertices(mesh->mNumVertices);
	std::vector<Vector3> normals(mesh->mNumVertices);
	std::vector<Vector2> uvs(mesh->mNumVertices);
	std::vector<unsigned int> indices(3 * mesh->mNumFaces);

	std::vector<unsigned int> remap(3 * mesh->mNumFaces);

	for (unsigned int vi = 0; vi < mesh->mNumVertices; vi++)
	{
		vertices[vi].x = mesh->mVertices[vi].x;
		vertices[vi].y = mesh->mVertices[vi].y;
		vertices[vi].z = mesh->mVertices[vi].z;
	}

	bool hasBadFace = false;
	unsigned int vaildFaceNum = 0;
	for (unsigned int fi = 0; fi < mesh->mNumFaces; fi++)
	{
		if (mesh->mFaces[fi].mNumIndices != 3)
		{
            hasBadFace = true;
			break;
		}

		indices[3 * vaildFaceNum + 0] = mesh->mFaces[fi].mIndices[0];
		indices[3 * vaildFaceNum + 1] = mesh->mFaces[fi].mIndices[1];
		indices[3 * vaildFaceNum + 2] = mesh->mFaces[fi].mIndices[2];

		vaildFaceNum++;
	}

	if (hasBadFace || vaildFaceNum == 0) return false;
	indices.resize(3 * vaildFaceNum);

	if(mesh->HasNormals())
    {
        for (unsigned int ni = 0; ni < mesh->mNumVertices; ni++)
        {
			normals[ni].x = mesh->mNormals[ni].x;
			normals[ni].y = mesh->mNormals[ni].y;
			normals[ni].z = mesh->mNormals[ni].z;
		}
	}

	// uvs 向量按顶点数预分配，所以「无 UV」的网格在下方重建分支里也会得到一条全 0
	// 的 UV 通道（历史行为，TileBuilder 同样按此语义补零）——重建时必须同步声明
	// 分量数，否则违反 assimp 不变式，见下。
	if (mesh->HasTextureCoords(0))
	{
		for(unsigned int vi = 0; vi < mesh->mNumVertices; vi++)
        {
			uvs[vi].x = mesh->mTextureCoords[0][vi].x;
			uvs[vi].y = mesh->mTextureCoords[0][vi].y;
		}
	}

	MergeVertices(indices, vertices, normals, uvs);  // modifies vectors in-place

	float weights[] = { optimizerItem.nweight, optimizerItem.nweight, optimizerItem.nweight };
	const size_t simpliedIndicesNum = meshopt_simplifyWithAttributes(indices.data(), indices.data(), indices.size(),
		(float*)vertices.data(), vertices.size(), vertex_size,
		(float*)normals.data(), normal_size, weights, 3,
		NULL, size_t(optimizerItem.threshold * indices.size()), optimizerItem.error, GetSimplificationOptions(optimizerItem));

	if (simpliedIndicesNum > 0)
	{
		indices.resize(simpliedIndicesNum);
		// GPU vertex cache optimization (reuse post-T&L vertices)
		meshopt_optimizeVertexCache(indices.data(), indices.data(),
		                             simpliedIndicesNum, vertices.size());
		// Fragment overdraw optimization (reduce pixel shader waste)
		meshopt_optimizeOverdraw(indices.data(), indices.data(),
		                          simpliedIndicesNum,
		                          (const float*)vertices.data(),
		                          vertices.size(), vertex_size, 1.05f);
	}

	size_t optimizerVerticesNum = reorder ? KeepEffectiveVertices(indices, vertices, normals, uvs) : vertices.size();

	mesh->mNumVertices = (unsigned int)optimizerVerticesNum;
	
	delete[] mesh->mVertices; mesh->mVertices = new aiVector3D[optimizerVerticesNum];
	for (unsigned int vi = 0; vi < optimizerVerticesNum; vi++)
		mesh->mVertices[vi] = aiVector3D(vertices[vi].x, vertices[vi].y, vertices[vi].z);

	mesh->mNumFaces = (unsigned int)simpliedIndicesNum / 3;
    delete[] mesh->mFaces; mesh->mFaces = new aiFace[mesh->mNumFaces];
	for (unsigned int fi = 0; fi < mesh->mNumFaces; fi++)
	{
		aiFace& localFace = mesh->mFaces[fi];
		localFace.mNumIndices = 3;
		localFace.mIndices = new unsigned int[3];
		localFace.mIndices[0] = indices[3 * fi + 0];
		localFace.mIndices[1] = indices[3 * fi + 1];
		localFace.mIndices[2] = indices[3 * fi + 2];
	}
	
	if (optimizerVerticesNum != 0 && normals.size() == optimizerVerticesNum)
	{
		delete[] mesh->mNormals; mesh->mNormals = new aiVector3D[optimizerVerticesNum];
		for (unsigned int vi = 0; vi < optimizerVerticesNum; vi++)
			mesh->mNormals[vi] = aiVector3D(normals[vi].x, normals[vi].y, normals[vi].z);
	}

	if (optimizerVerticesNum != 0 && uvs.size() == optimizerVerticesNum)
	{
		delete[] mesh->mTextureCoords[0]; mesh->mTextureCoords[0] = new aiVector3D[optimizerVerticesNum];
		for (unsigned int vi = 0; vi < optimizerVerticesNum; vi++)
			mesh->mTextureCoords[0][vi] = aiVector3D(uvs[vi].x, uvs[vi].y, 0);

		// 我们写入的是二维 UV，必须同步声明分量数：导入器给「无 UV」网格留下的
		// mNumUVComponents[0] == 0，而 assimp 的不变式是「mTextureCoords[i] 非空
		// ⟹ mNumUVComponents[i] ≥ 1」。FBX 导出器按
		// `uv_data[uvi].size() / mNumUVComponents[uvi]` 做整数除法 → 除零 SIGFPE：
		// 修复前 `mesh -i any.obj -o out.fbx` 必然崩溃。
		mesh->mNumUVComponents[0] = 2;
	}

	delete[] mesh->mTangents;  mesh->mTangents = NULL;
	delete[] mesh->mBitangents;  mesh->mBitangents = NULL;

	return true;
}
