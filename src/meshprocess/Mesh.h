// ----------------------------------------------------------------
// A Mesh class for the Advanced Graphics
// course of the Master Game Dev
// ----------------------------------------------------------------

#ifndef ADG_MESH_H
#define ADG_MESH_H

#include <vector>
#include "../Vector3.h"

namespace adg {

	struct Index_Data {
		int vertex_idx{};
		int normal_idx{};
		int uv_idx{};
	};

	struct Face_Data {
		std::string mtl_name{};
		int smoothing_group{};
		std::vector<Index_Data> corners{};
	};

	// unified data
	struct Vertex_DataGPU {
		Vector3 position{};
		Vector3 normal{};
		// Vector2 uv;
	};

	struct Mesh_Data {
		std::string name{};
		std::string mtl{};
		std::vector<Vector3> vertices{};
		//std::vector<Vector3> uvs{};
		std::vector<Vector3> normals{};
		std::vector<Face_Data> faces{};
		int vertex_Count{};
		int uvs_Count{};
		int normals_Count{};
		int face_Count{};
	};

	class Transform;
	class Quaternion;

	class Mesh {

	public:
		Mesh_Data m_data{};
		
		Mesh();
		Mesh(const Mesh& other) = delete;

		bool hasNormals() const;
		bool hasUvs() const;

		bool applyTransform(const Transform& trs);
	};
}

#endif !ADG_MESH_H