
#include "Mesh.h"

glm::vec3 HEdge::GetDirection() const
{
	return next->tail->position - tail->position;
}

HEdge* HEdge::Prev()
{
	int n = 0;
	HEdge* e = face->edge;
	do {
		n++;
		e = e->next;
	} while (e != face->edge);

	HEdge* prev = this;
	for (int i = 1; i < n; i++)
		prev = prev->next;

	return prev;
}

void HFace::CalculateNormal()
{
	normal = glm::cross(edge->GetDirection(), edge->next->GetDirection());
	normal = glm::normalize(normal);
}

void HMesh::RemoveObselete()
{
	std::vector<HVertex*> tempV;
	std::vector<HEdge*> tempE;
	std::vector<HFace*> tempF;

	bool remove = false;
	int id = 0;
	for (auto v : vertices)
	{
		for (auto i : vids)
		{
			if (v->id == i)
				remove = true;
		}
		if (!remove)
		{
			v->id = ++id;
			tempV.push_back(v);
		}
		remove = false;
	}
	vertices = tempV;

	remove = false;
	id = 0;
	for (auto e : edges)
	{
		for (auto i : eids)
		{
			if (e->id == i)
				remove = true;
		}
		if (!remove)
		{
			e->id = ++id;
			tempE.push_back(e);
		}
		remove = false;
	}
	edges = tempE;

	remove = false;
	id = 0;
	for (auto f : faces)
	{
		for (auto i : fids)
		{
			if (f->id == i)
				remove = true;
		}
		if (!remove)
		{
			f->id = ++id;
			tempF.push_back(f);
		}
		remove = false;
	}
	faces = tempF;

	vids.clear();
	eids.clear();
	fids.clear();
}

void HMesh::Scale(const glm::vec3& scale)
{
	for (auto v : vertices)
	{
		v->position.x *= scale.x;
		v->position.y *= scale.y;
		v->position.z *= scale.z;
	}
}

bool HMesh::AreCoplanar(HFace* f1, HFace* f2)
{
	const static float e = 0.01;
	float diff = 1.0f - glm::dot(f1->normal, f2->normal);

	return (diff == 0.0f || diff < e);
}

void HMesh::FixTopological(HEdge* in, HEdge* out)
{
	if (in->twin->face == out->twin->face)
	{
		int nVerts = 0;
		HFace* adjFace = in->twin->face;
		HEdge* e = adjFace->edge;
		do {
			nVerts++;
			e = e->next;
		} while (e != adjFace->edge);

		if (nVerts == 3)
		{
			HEdge* third = in->twin->next;
			out->face->edge = third;
			third->face = in->face;
			HEdge* inPrev = in->prev;
			inPrev->next = third;
			third->next = out->next;
			// is this really neccessary?
			out->next->prev = third;
			third->prev = in->prev;

			fids.push_back(adjFace->id);
			eids.push_back(in->id);			in->dirty = true;
			eids.push_back(in->twin->id);	in->twin->dirty = true;
			eids.push_back(out->id);		out->dirty = true;
			eids.push_back(out->twin->id);	out->twin->dirty = true;
			vids.push_back(out->tail->id);
		}
		else
		{
			out->face->edge = out->next;

			in->twin->tail = out->twin->tail;
			in->next = out->next;
			in->next->prev = in;
			in->twin->prev = out->twin->prev;

			adjFace->edge = in->twin->next;
			out->twin->prev->next = in->twin;
			out->twin->prev->next->twin = in;

			eids.push_back(out->id);		out->dirty = true;
			eids.push_back(out->twin->id);	out->twin->dirty = true;
			vids.push_back(out->tail->id);
		}
	}
	else
	{
		in->face->edge = in;
	}
}

int HMesh::MergeFaces()
{
	int n = 0;
	HEdge* e;
	for (int i = 0; i < edges.size() / 2; i++)
	{
		e = edges[i];

		if (e->dirty)	// should this happen?
			continue;

		if (AreCoplanar(e->face, e->twin->face))
		{
			n++;
			
			HFace* adjFace = e->twin->face;
			fids.push_back(adjFace->id);

			HEdge* edgePrev = e->prev;
			HEdge* edgeNext = e->next;
			HEdge* twinPrev = e->twin->prev;
			HEdge* twinNext = e->twin->next;

			HEdge* curr = twinNext;
			do {
				curr->face = e->face;
				curr = curr->next;
			} while (curr != e->twin);

			edgePrev->next = twinNext;	
			twinPrev->next = edgeNext;
			edgeNext->prev = twinPrev;
			twinNext->prev = edgePrev;

			eids.push_back(e->id);			e->dirty = true;
			eids.push_back(e->twin->id);	e->twin->dirty = true;

			FixTopological(twinPrev, edgeNext);
			FixTopological(edgePrev, twinNext);
		}
	}

	RemoveObselete();

	return n;
}

void HMesh::GetTriangleIndices(std::vector<int>& indices) const
{
	for (HFace* f : faces)
	{
		HEdge* start = f->edge;
		HEdge* e = start;
		int first, second, third;
		first = e->tail->id - 1;
		do		// triangle fan
		{
			second = e->next->tail->id - 1;
			e = e->next;
			third = e->next->tail->id - 1;

			indices.push_back(first);
			indices.push_back(second);
			indices.push_back(third);
		} while (e->next->next != start);
	}
}

void HMesh::GetLineIndices(std::vector<int>& indices) const
{
	int id1, id2;
	for (int i = 0; i < edges.size() / 2; i++)
	{
		id1 = edges[i]->tail->id - 1;
		id2 = edges[i]->next->tail->id - 1;

		indices.push_back(id1);
		indices.push_back(id2);
	}
}

void HMesh::GetModelData(ModelData& m) const
{
	m.vertices.clear();
	m.indices.clear();
	m.frameIndices.clear();

	for (auto v : vertices)
	{
		m.vertices.push_back(v->position);
	}
	GetTriangleIndices(m.indices);
	GetLineIndices(m.frameIndices);
}