// ----------------------------------------------------------------
// A Mesh class for the Advanced Graphics
// course of the Master Game Dev
// ----------------------------------------------------------------

#include "Mesh.h"
#include "../Transform.h"
#include <fstream>
#include <sstream>

adg::Mesh::Mesh() {}

bool adg::Mesh::hasNormals() const
{
	return m_data.normals_Count > 0;
}

bool adg::Mesh::hasUvs() const
{
	return m_data.uvs_Count > 0;
}

bool adg::Mesh::applyTransform(const Transform& trs)
{
	// apply transform to vertices
	auto end = m_data.vertices.end();
	for (std::vector<Vector3>::iterator i = m_data.vertices.begin();
		i != end;
		++i)
	{
		*i = trs.apply_to_point(*i);
	}

	// apply transform to normals
	end = m_data.normals.end();
	for (std::vector<Vector3>::iterator it = m_data.normals.begin();
		it != end;
		++it)
	{
		*it = trs.apply_to_versor(*it);
	}

	return false;
}
