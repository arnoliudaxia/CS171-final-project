
#pragma once

#include <iostream>
#include "Mesh.h"

struct ObjFace
{
	int id;
	std::vector<int> vids;
};

bool ParseObj(const std::string& file, HMesh& mesh, bool merge = false);

bool ParseObj(const std::string& file, std::vector<HMesh>& meshes, bool merge = false);

// check for bad/invalid file
bool Validate(const std::string& file);

void ParseLine(std::vector<ObjFace>& objFaces, HMesh& mesh, const std::string& line, bool& split, bool& first, int offset = 0);

void AddFace(HMesh& mesh, const std::vector<int>& vids);