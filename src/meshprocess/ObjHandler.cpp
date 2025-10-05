// ----------------------------------------------------------------
// ObjHandler class to handle import and export of meshes
// for the Advanced Graphics course of the Master Game Dev
// ----------------------------------------------------------------

#include "ObjHandler.h"
#include "Mesh.h"
#include "tiny_obj_loader.h"
#include <fstream>
#include <sstream>
#include <iostream>

bool adg::ObjHandler::load(const std::string& file_path, Mesh& outMesh)
{
	std::ifstream file(file_path);
	if (!file.is_open()) {
		std::cerr << "[Error]: failed to open " << file_path << "\n";
		return false;
	}

	// clear data
	outMesh.m_data = {};

	// temp data
	int smoothing{};
	std::string line{};
	std::string currentMtl{};
	int lineNumber{};

	// face parsing
	auto parseFaceToken = [](const std::string& token) -> Index_Data {
		
		Index_Data idx{};
		size_t firstSlash = token.find('/');
		size_t secondSlash = token.find('/', firstSlash + 1);

		// Obj face formats:
		// Only vertex index: f 1 2 3
		// Vertex/texture: f 1/1 2/2 3/3
		// Vertex/texture/normals: f 1/1/1 2/2/2 3/3/3

		// Vertex index
		if (firstSlash != std::string::npos) // found slash
			idx.vertex_idx = std::stoi(token.substr(0, firstSlash)) - 1; // substr(initialPos, length)
		else
			idx.vertex_idx = std::stoi(token) - 1; //only vertex case

		// UV index
		if (firstSlash != std::string::npos && secondSlash != firstSlash + 1) {
			idx.uv_idx = std::stoi(token.substr(firstSlash + 1, secondSlash - firstSlash - 1)) - 1; 
		}

		// Normal index
		if (secondSlash != std::string::npos) {
			idx.normal_idx = std::stoi(token.substr(secondSlash + 1)) - 1;
		}

		return idx;
		};

	// parsing
	while (std::getline(file, line))
	{
		++lineNumber;

		// ignore comments and empty lines
		if (line.empty() || line[0] == '#')
			continue;

		// get prefix
		std::istringstream iss(line);
		std::string prefix;
		iss >> prefix;

		// evaluate prefix
		if (prefix == "mtllib") {
			iss >> outMesh.m_data.mtl;
		}
		else if (prefix == "usemtl") {
			iss >> currentMtl;
		}
		else if (prefix == "o") // mesh name
		{
			iss >> outMesh.m_data.name;
		}
		else if (prefix == "s")  // smoothing group
		{
			std::string token{};
			iss >> token;
			smoothing = (token == "off" ? 0 : std::stoi(token));
		}
		else if (prefix == "v") // vertex data
		{
			Vector3 vtx{};
			iss >> vtx.x >> vtx.y >> vtx.z;
			outMesh.m_data.vertices.push_back(vtx);
		}
		else if (prefix == "vt")  // uv data
		{
			// Vector2 uv{};
			// iss >> uv.x >> uv.y;
			// outMesh.m_data.uvs.push_back(uv);
		}
		else if (prefix == "vn")  // normal data
		{
			Vector3 normal{};
			iss >> normal.x >> normal.y >> normal.z;
			outMesh.m_data.normals.push_back(normal);
		}
		else if (prefix == "f")  // face data
		{
			Face_Data fData;
			fData.smoothing_group = smoothing;
			fData.mtl_name = currentMtl;
			std::string token;

			while (iss >> token)
			{
				fData.corners.push_back(parseFaceToken(token));
			}
			outMesh.m_data.faces.push_back(fData);
		}
		else
		{
			std::cerr << "[Warning][Line " << lineNumber << "]: unknown OBJ prefix '" << prefix << "'\n";
			// return false;
		}
	}

	outMesh.m_data.vertex_Count = static_cast<int>(outMesh.m_data.vertices.size());
	outMesh.m_data.normals_Count = static_cast<int>(outMesh.m_data.normals.size());
	outMesh.m_data.face_Count = static_cast<int>(outMesh.m_data.faces.size());

	return true;
}

bool adg::ObjHandler::save(const std::string& file_path, const std::string& file_name, const Mesh& mesh)
{
	std::ofstream objFile(file_path + "/" + file_name + ".obj");
	std::ofstream mtlFile(file_path + "/" + file_name + ".mtl");

	if (!objFile.is_open() || !mtlFile.is_open()) {
		std::cerr << "[Error] Cannot open output files for writing\n";
		return false;
	}

	objFile << "# exported from adg::ObjHandler\n";
	objFile << "mtllib " << file_name << ".mtl\n";

	// Write object name
	objFile << "o " << mesh.m_data.name << "\n";

	// Write vertices
	for (const Vector3& v : mesh.m_data.vertices) {
		objFile << "v " << v.x << " " << v.y << " " << v.z << "\n";
	}

	// Write uvs
	if (mesh.hasUvs())
	{

	}

	// Write normals
	if (mesh.hasNormals())
	{
		for (const Vector3& n : mesh.m_data.normals) {
			objFile << "vn " << n.x << " " << n.y << " " << n.z << "\n";
		}
	}

	// Smoothing group (for now 0)
	objFile << "s 0" << "\n";

	// Use a simple material
	std::string materialName = "material_0";
	objFile << "usemtl " << materialName << "\n";

	//  Write faces
	for (const Face_Data& face : mesh.m_data.faces) {
		objFile << "f";
		for (const Index_Data& idx : face.corners) {

			// vertex index
			objFile << " " << (idx.vertex_idx + 1);
			
			if (mesh.hasUvs()) // uv index
			{
				objFile << "/" << (idx.uv_idx + 1);
			}
			if (mesh.hasNormals() && !mesh.hasUvs()) // normal index without uv
			{
				objFile << "/";
			}
			if (mesh.hasNormals()) // normal index
			{
				objFile << "/" << (idx.normal_idx + 1);
			}
		}
		objFile << "\n";
	}

	Vector3 color{ Scalar(1), Scalar(0), Scalar(0) };

	// Write inside material file
	mtlFile << "newmtl " << materialName << "\n";
	mtlFile << "Kd " << color.x << " " << color.y << " " << color.z << "\n";
	mtlFile << "Ka 0 0 0\n";     // ambient color
	mtlFile << "Ks 0 0 0\n";     // specular
	mtlFile << "d 1\n";          // opacity
	mtlFile << "illum 1\n\n";    // lighting model

	objFile.close();
	mtlFile.close();

	std::cout << "\nOBJ and MTL successful exported in: " << file_path << "\n";

	return true;
}

bool adg::ObjHandler::loadWithTinyObjLoader(const std::string& file_path, Mesh& outMesh)
{
	tinyobj::attrib_t attrib{};

	// shapes is the number of loaded mesh/sub-meshes
	std::vector<tinyobj::shape_t> shapes{};
	std::vector<tinyobj::material_t> materials{};
	std::string warn{};
	std::string err{};

	// Load Obj
	bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, file_path.c_str());
	if (!warn.empty()) {
		std::cout << "tinyobj warning: " << warn << std::endl;
	}
	if (!err.empty()) {
		std::cerr << "tinyobj error: " << err << std::endl;
	}
	if (!ret) {
		std::cerr << "Failed to load/parse .obj.\n";
		return false;
	}

	// Debug
	std::cout << "**DEBUG**" << std::endl;
	std::cout << "shapes: " << shapes.size() << std::endl;
	std::cout << "name: " << shapes[0].name.c_str() << std::endl;
	std::cout << "indices: " << shapes[0].mesh.indices.size() << std::endl;
	std::cout << "indices/3: " << shapes[0].mesh.indices.size() / 3 << std::endl;
	std::cout << "num face vertices: " << shapes[0].mesh.num_face_vertices.size() << std::endl;
	std::cout << "points.indices: " << shapes[0].points.indices.size() << std::endl;

	std::cout << "num_vertices: " << attrib.vertices.size() << std::endl;
	std::cout << "num_vertices/3: " << attrib.vertices.size() / 3 << std::endl;
	// v 0.437500 0.164063 0.765625 - vertex 1 in file .obj
	std::cout << "vertices[0]: " << attrib.vertices[0] << std::endl;
	std::cout << "vertices[1]: " << attrib.vertices[1] << std::endl;
	std::cout << "vertices[2]: " << attrib.vertices[2] << std::endl;

	if (attrib.normals.size() > 0)
	{
		std::cout << "normal[0]: " << attrib.normals[0] << std::endl;
		std::cout << "normal[1]: " << attrib.normals[1] << std::endl;
		std::cout << "normal[2]: " << attrib.normals[2] << std::endl;
	}

	// attrib.GetVertices()
	std::cout << "num_colors: " << attrib.colors.size() << std::endl;
	std::cout << "num_normals: " << attrib.normals.size() / 3 << std::endl;
	std::cout << "num_texcoords: " << attrib.texcoords.size() / 2 << std::endl;

	// face debug
	std::cout << "first face" << std::endl;
	std::cout << "vertex index0: " << shapes[0].mesh.indices[0].vertex_index << std::endl;
	std::cout << "normal index0: " << shapes[0].mesh.indices[0].normal_index << std::endl;
	std::cout << "texcoord index0: " << shapes[0].mesh.indices[0].texcoord_index << std::endl;
	std::cout << "vertex index1: " << shapes[0].mesh.indices[1].vertex_index << std::endl;
	std::cout << "normal index1: " << shapes[0].mesh.indices[1].normal_index << std::endl;
	std::cout << "texcoord index1: " << shapes[0].mesh.indices[1].texcoord_index << std::endl;
	std::cout << "vertex index2: " << shapes[0].mesh.indices[2].vertex_index << std::endl;
	std::cout << "normal index2: " << shapes[0].mesh.indices[2].normal_index << std::endl;
	std::cout << "texcoord index2: " << shapes[0].mesh.indices[2].texcoord_index << std::endl;
	std::cout << "--------------------------------------" << std::endl;

	return true;
}
