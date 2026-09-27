#include "GeometryProcessor.h"

#include "cinder/Log.h"
#include "cinder/TriMesh.h"

#include <array>
#include <cctype>
#include <cstring>
#include <map>
#include <unordered_map>

namespace fbximp {

namespace {

//------------------------------------------------------------------------------
// Smooth normal recomputation over an expanded-corner mesh: weld by exact
// position, accumulate area-weighted face normals, scatter back.
//------------------------------------------------------------------------------

struct Vec3Hash {
    size_t operator()(const ci::vec3& v) const {
        size_t h = std::hash<float>()(v.x);
        h ^= std::hash<float>()(v.y) + 0x9e3779b9u + (h << 6) + (h >> 2);
        h ^= std::hash<float>()(v.z) + 0x9e3779b9u + (h << 6) + (h >> 2);
        return h;
    }
};
struct Vec3Eq {
    bool operator()(const ci::vec3& a, const ci::vec3& b) const {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }
};

void recomputeSmoothNormals(ProcessedGeometry& g) {
    if (g.indices.empty()) return;
    std::unordered_map<ci::vec3, uint32_t, Vec3Hash, Vec3Eq> weld;
    weld.reserve(g.positions.size());
    std::vector<uint32_t> weldedIndexOf(g.positions.size());
    std::vector<ci::vec3> weldedPos;
    for (size_t i = 0; i < g.positions.size(); i++) {
        auto it = weld.find(g.positions[i]);
        if (it == weld.end()) {
            uint32_t idx = (uint32_t)weldedPos.size();
            weld[g.positions[i]] = idx;
            weldedPos.push_back(g.positions[i]);
            weldedIndexOf[i] = idx;
        } else {
            weldedIndexOf[i] = it->second;
        }
    }

    std::vector<ci::vec3> normals(weldedPos.size(), ci::vec3(0.f));
    for (size_t t = 0; t + 2 < g.indices.size(); t += 3) {
        uint32_t ia = weldedIndexOf[g.indices[t]];
        uint32_t ib = weldedIndexOf[g.indices[t + 1]];
        uint32_t ic = weldedIndexOf[g.indices[t + 2]];
        ci::vec3 n = cross(weldedPos[ib] - weldedPos[ia], weldedPos[ic] - weldedPos[ia]);
        normals[ia] += n;
        normals[ib] += n;
        normals[ic] += n;
    }

    g.normals.resize(g.positions.size());
    for (size_t i = 0; i < g.positions.size(); i++) {
        ci::vec3 n = normals[weldedIndexOf[i]];
        float len = length(n);
        g.normals[i] = len > 1e-12f ? n / len : ci::vec3(0.f, 1.f, 0.f);
    }
}

//------------------------------------------------------------------------------
// Baking a mesh into a geometry accumulator
//------------------------------------------------------------------------------

struct BakeAccumulator {
    std::vector<ci::vec3> positions;
    std::vector<ci::vec3> normals;    // empty until first contribution decides
    std::vector<ci::vec2> uvs;
    std::vector<uint32_t> indices;
    bool hasNormals = true;
    bool hasUvs = true;
    bool anyMissingNormals = false;   // mixed sources -> recompute all
    size_t tris = 0;
};

void appendBaked(BakeAccumulator& acc, const ImportMesh& mesh, bool bakeTransform) {
    if (acc.positions.empty()) {
        acc.hasNormals = !mesh.normals.empty();
        acc.hasUvs = !mesh.uvs.empty();
    }
    const bool meshNormals = !mesh.normals.empty();
    const bool meshUvs = !mesh.uvs.empty();
    if (acc.hasNormals && !meshNormals) acc.anyMissingNormals = true;

    const ci::mat4& m = mesh.worldMatrix;
    ci::mat3 rotM(m);                       // upper 3x3
    ci::mat3 normalM = transpose(inverse(rotM));
    const bool mirrored = determinant(rotM) < 0.f;

    uint32_t base = (uint32_t)acc.positions.size();
    for (size_t i = 0; i < mesh.positions.size(); i++) {
        ci::vec3 p = mesh.positions[i];
        ci::vec3 n = meshNormals ? mesh.normals[i] : ci::vec3(0.f, 1.f, 0.f);
        if (bakeTransform) {
            p = ci::vec3(m * ci::vec4(p, 1.f));
            n = normalize(normalM * n);
        }
        acc.positions.push_back(p);
        if (acc.hasNormals) acc.normals.push_back(meshNormals ? n : ci::vec3(0.f));
        if (acc.hasUvs) acc.uvs.push_back(meshUvs ? mesh.uvs[i] : ci::vec2(0.f));
    }
    for (size_t t = 0; t < mesh.indices.size(); t += 3) {
        uint32_t a = base + mesh.indices[t];
        uint32_t b = base + mesh.indices[t + 1];
        uint32_t c = base + mesh.indices[t + 2];
        if (bakeTransform && mirrored) std::swap(b, c);  // keep faces front-facing
        acc.indices.insert(acc.indices.end(), { a, b, c });
        acc.tris++;
    }
}

ProcessedGeometry finalizeGeometry(std::string name, int materialIndex, int nodeIndex,
                                   BakeAccumulator acc) {
    ProcessedGeometry g;
    g.name = std::move(name);
    g.materialIndex = materialIndex;
    g.nodeIndex = nodeIndex;
    g.inputVerts = acc.positions.size();
    g.inputTris = acc.tris;
    g.positions = std::move(acc.positions);
    g.indices = std::move(acc.indices);
    if (acc.hasNormals) g.normals = std::move(acc.normals);
    if (acc.hasUvs) g.uvs = std::move(acc.uvs);

    if (g.normals.size() != g.positions.size() || acc.anyMissingNormals)
        recomputeSmoothNormals(g);          // source had missing/mixed normals
    if (g.uvs.size() != g.positions.size())
        g.uvs.clear();                      // partial UVs -> drop entirely

    return g;
}

std::string sanitizeName(const std::string& s) {
    std::string out;
    for (char c : s) {
        if (std::isalnum((unsigned char)c) || c == '_' || c == '-') out += c;
        else out += '_';
    }
    if (out.empty()) out = "unnamed";
    return out;
}

} // namespace

void finalizeAttributes(ProcessedGeometry& g) {
    if (g.indices.empty() || g.normals.size() != g.positions.size()
        || g.uvs.size() != g.positions.size())
        return;

    ci::TriMesh mesh(ci::TriMesh::Format().positions().normals().texCoords(2)
        .tangents().bitangents());
    mesh.appendPositions(g.positions.data(), g.positions.size());
    mesh.appendNormals(g.normals.data(), g.normals.size());
    mesh.appendTexCoords0(g.uvs.data(), g.uvs.size());
    mesh.appendIndices(g.indices.data(), (uint32_t)g.indices.size());
    if (mesh.recalculateTangents() && mesh.recalculateBitangents()) {
        g.tangents = mesh.getTangents();
        g.bitangents = mesh.getBitangents();
    } else {
        g.tangents.clear();
        g.bitangents.clear();
        CI_LOG_W("Tangent generation failed for '" << g.name << "' (degenerate UVs?)");
    }
}

//==============================================================================
// Mode B — merge by material, bake transforms
//==============================================================================

ProcessedScene mergeByMaterial(const ImportScene& scene) {
    ProcessedScene result;
    std::map<int, BakeAccumulator> perMaterial;
    std::map<int, std::string> firstName;

    for (const ImportMesh& mesh : scene.meshes) {
        auto& acc = perMaterial[mesh.materialIndex];
        if (firstName.find(mesh.materialIndex) == firstName.end())
            firstName[mesh.materialIndex] = mesh.name;
        appendBaked(acc, mesh, /*bakeTransform=*/true);
    }

    for (auto& [matIdx, acc] : perMaterial) {
        if (acc.indices.empty()) continue;
        const std::string& matName = scene.materials[matIdx].name;
        ProcessedGeometry g = finalizeGeometry(
            sanitizeName(matName), matIdx, -1, std::move(acc));
        CI_LOG_I("Merged by material '" << matName << "': " << g.inputTris << " tris, "
            << g.inputVerts << " verts (baked transforms)");
        result.geometries.push_back(std::move(g));
    }
    return result;
}

//==============================================================================
// Mode A — keep hierarchy, merge only identical-(material, world) groups
//==============================================================================

ProcessedScene mergeKeepingHierarchy(const ImportScene& scene) {
    ProcessedScene result;
    result.nodes = scene.nodes;

    // group key: material index + exact world matrix bits
    struct GroupKey {
        int material;
        std::array<float, 16> matrix;
        bool operator<(const GroupKey& o) const {
            if (material != o.material) return material < o.material;
            return matrix < o.matrix;
        }
    };
    std::map<GroupKey, BakeAccumulator> groups;
    std::map<GroupKey, int> groupNode;      // representative node index

    for (const ImportMesh& mesh : scene.meshes) {
        GroupKey key{ mesh.materialIndex, {} };
        // bit-exact copy of the world matrix
        std::memcpy(key.matrix.data(), value_ptr(mesh.worldMatrix), sizeof(float) * 16);
        auto& acc = groups[key];
        if (groupNode.find(key) == groupNode.end()) groupNode[key] = mesh.nodeIndex;
        appendBaked(acc, mesh, /*bakeTransform=*/false);
    }

    for (auto& [key, acc] : groups) {
        if (acc.indices.empty()) continue;
        const std::string& matName = scene.materials[key.material].name;
        ProcessedGeometry g = finalizeGeometry(
            sanitizeName(matName), key.material, groupNode[key], std::move(acc));
        CI_LOG_I("Hierarchy merge '" << matName << "' @node " << g.nodeIndex << ": "
            << g.inputTris << " tris, " << g.inputVerts << " verts");
        result.geometries.push_back(std::move(g));
    }
    return result;
}

} // namespace fbximp
