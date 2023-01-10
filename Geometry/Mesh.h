
#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "Body.h"

// half-edge data structure to represent mesh collision geometry
// for fast adjacency queries

struct HEdge;
struct HFace;

// represents a vertex on the mesh
struct HVertex
{
	glm::vec3 position;		// vertex coordinate position
	HEdge* edge;			// the edge pointing to this vertex
	int id;					// vertex index
};

// represents an edge on the mesh
struct HEdge
{
	HVertex* tail;			// vertex pointing to this edge
	HFace* face;			// face to the left of this edge (CCW winding)
	HEdge* next;			// next edge belonging to the same face
	HEdge* prev;
	HEdge* twin;			// opposite edge pointing to the tail
	int id;					// edge index
	bool duplicate;			// marked false for original edge, and true for the twin
	bool dirty = false;

	glm::vec3 GetDirection() const;

	HEdge* Prev();
};

// represents a face on the mesh
struct HFace
{
	HEdge* edge;			// any edge on this face
	glm::vec3 normal;		//face normal should be pointing outwards;
	int id;					// face index

	void CalculateNormal();
};

struct HMesh
{
	std::vector<HVertex*> vertices;
	std::vector<HEdge*> edges;
	std::vector<HFace*> faces;

	// items to be deleted after face merging
	std::vector<int> vids;
	std::vector<int> eids;
	std::vector<int> fids;

	void RemoveObselete();

	void Scale(const glm::vec3& scale);

	bool AreCoplanar(HFace* f1, HFace* f2);

	// fix topological invaraints violated while merging
	// each vertex should have at least 3 adjacent faces
	void FixTopological(HEdge* in, HEdge* out);

	// calculates the triangle indices
	// used by gaphics to render solid mesh in triangle mode
	void GetTriangleIndices(std::vector<int>& indices) const;

	// calculates line indices
	// used by graphics to render mesh in wire-frame mode
	void GetLineIndices(std::vector<int>& indices) const;

	void GetModelData(ModelData& m) const;
};