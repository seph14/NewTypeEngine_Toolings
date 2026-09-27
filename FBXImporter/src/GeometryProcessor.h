#pragma once

#include "FbxImporter.h"

namespace fbximp {

// Final geometry ready for serialization. Vertex arrays are parallel;
// tangents/bitangents are filled when normals+uvs exist.
struct ProcessedGeometry {
    std::string name;
    int materialIndex = -1;
    int nodeIndex = -1;                  // hierarchy mode: attachment node
    std::vector<ci::vec3> positions;
    std::vector<ci::vec3> normals;
    std::vector<ci::vec2> uvs;
    std::vector<ci::vec3> tangents;
    std::vector<ci::vec3> bitangents;
    std::vector<uint32_t> indices;
    size_t inputVerts = 0;               // stats before merge/dedup
    size_t inputTris = 0;
};

struct ProcessedScene {
    std::vector<ProcessedGeometry> geometries;
    std::vector<ImportNode> nodes;       // hierarchy mode only (empty otherwise)
};

// Mode B: bake every world transform into the vertices (normal matrix for
// normals/tangents, winding flip for mirrored transforms) and merge into one
// geometry per material.
ProcessedScene mergeByMaterial(const ImportScene& scene);

// Mode A: keep the (pruned) node tree; merge only meshes that share the same
// material AND the exact same world matrix — geometry itself is untouched.
ProcessedScene mergeKeepingHierarchy(const ImportScene& scene);

// Compute tangents/bitangents on the final (post-merge, post-optimize)
// topology. Requires normals + uvs; no-op otherwise.
void finalizeAttributes(ProcessedGeometry& g);

} // namespace fbximp
