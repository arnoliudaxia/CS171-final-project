
#include "ObjParser.h"
#include <fstream>
#include <string>
#include <sstream>
#include <map>

bool ParseObj(const std::string& file, HMesh& mesh, bool merge)
{
	mesh.vertices.clear();
	mesh.edges.clear();
	mesh.faces.clear();

	if (!Validate(file))
		return false;

	std::ifstream ifs(file, std::ifstream::in);
	std::string line;
	std::vector<ObjFace> objFaces;

	bool split = false;
	bool first = true;
	while (ifs.good())
	{
		getline(ifs, line);
		ParseLine(objFaces, mesh, line, split, first);
	}

	for (ObjFace objFace : objFaces)
	{
		AddFace(mesh, objFace.vids);
	}

	ConnectTwins(mesh);
	SortEdges(mesh);

	if (merge)
	{
		int n = mesh.MergeFaces();
		SortEdges(mesh);
	}
}

bool ParseObj(const std::string& file, std::vector<HMesh>& meshes, bool merge)
{
	meshes.clear();

	if (!Validate(file))
		return false;

	std::ifstream ifs(file, std::ifstream::in);
	std::string line;
	std::vector<ObjFace> objFaces;

	bool split = false;
	bool first = true;
	int offset = 0;
	while (ifs.good())
	{
		HMesh mesh;
		while (ifs.good())
		{
			getline(ifs, line);
			ParseLine(objFaces, mesh, line, split, first, offset);
			if (split)
				break;
		}

		for (ObjFace objFace : objFaces)
		{
			AddFace(mesh, objFace.vids);
		}
		objFaces.clear();

		ConnectTwins(mesh);
		SortEdges(mesh);
		offset += mesh.vertices.size();

		if (merge)
		{
			int n = mesh.MergeFaces();
			SortEdges(mesh);
		}
		
		meshes.push_back(mesh);
		split = false;
	}
}

bool Validate(const std::string& file)
{
	std::string ext = file.substr(file.find_last_of('.') + 1);

	if (ext != "obj")
		return false;

	std::ifstream ifs(file, std::ifstream::in);
	if (!ifs.good())
		return false;

	return true;
}

void ParseLine(std::vector<ObjFace>& objFaces, HMesh& mesh, const std::string& line, bool& split, bool& first,int offset)
{
	if (line[0] == 'v' && line[1] == ' ')
	{
		first = false;
		HVertex* v = new HVertex();

		sscanf(line.c_str(), "v %f %f %f", &v->position.x, &v->position.y, &v->position.z);
		v->id = mesh.vertices.size() + 1;

		mesh.vertices.push_back(v);
		v = nullptr;
	}
	else if (line[0] == 'f' && line[1] == ' ')
	{
		std::stringstream tokenizer(line);
		std::vector<std::string> tokens;
		std::string token;

		while (getline(tokenizer, token, ' '))
			tokens.push_back(token);

		struct ObjFace f;
		f.id = objFaces.size() + 1;
		int vid;
		for (int i = 1; i < tokens.size(); i++)
		{
			sscanf(tokens[i].c_str(), "%d", &vid);
			f.vids.push_back(vid - offset);
		}
		objFaces.push_back(f);
	}
	else if (line[0] == 'o' && line[1] == ' ' && !first)
	{
		split = true;
	}
}
