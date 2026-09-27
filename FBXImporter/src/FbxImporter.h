#pragma once

#include "cinder/Cinder.h"
#include "cinder/Matrix.h"
#include "cinder/Quaternion.h"
#include "cinder/Vector.h"

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace fbximp {

namespace fs = std::filesystem;

//==============================================================================
// Raw import structures — plain data, no ufbx types past the loader
//==============================================================================

// One mesh per (node instance x material). Vertices are expanded per index
// (corner); exact duplicates are collapsed later by the meshoptimizer dedup
// pass, which restores an optimal shared-vertex layout.
struct ImportMesh {
    std::string name;
    int materialIndex = -1;   // into ImportScene::materials
    int nodeIndex = -1;       // into ImportScene::nodes
    ci::mat4 worldMatrix;     // ufbx node->geometry_to_world

    std::vector<ci::vec3> positions;
    std::vector<ci::vec3> normals;  // empty = not present in file
    std::vector<ci::vec2> uvs;      // empty = not present in file
    std::vector<uint32_t> indices;  // triangles

    size_t sourceTriangles = 0;
};

// Engine MaterialType values (include/newtype/render/Material.h) — kept in
// sync manually; the JSON schema matches MaterialPool::materialDataToJson.
enum class EngineMaterialType : uint32_t {
    Null = 0, Diffuse = 1, Conductor = 2, Dielectric = 3, Plastic = 4,
    Emissive = 5, Subsurface = 6, Clearcoat = 7, Sheen = 8, Anisotropy = 9,
    Iridescence = 10, ThinDielectric = 11, Unlit = 12, Fabric = 13,
};

// All scalar/vector fields the engine serializes (MaterialData minus texture
// indices, which the engine binds by folder naming convention).
struct EngineMaterialData {
    EngineMaterialType type = EngineMaterialType::Diffuse;
    ci::vec4 albedo{ 0.8f, 0.8f, 0.8f, 1.0f };
    ci::vec3 emission{ 0.f, 0.f, 0.f };
    float roughness = 0.5f;
    float metallic = 0.f;
    float ior = 1.5f;
    float alphacut = 0.f;
    float specular_tint = 0.f;
    float specular_trans = 0.f;
    float clearcoat = 0.f;
    float clearcoat_gloss = 0.5f;
    float sheen = 0.f;
    float sheen_tint = 0.f;
    float anisotropic = 0.f;
    float anisotropic_rot = 0.f;
    float flatness = 0.f;
    float diffuse_trans = 0.f;
    float fabric = 0.f;
    float iridescence = 0.f;
    float iridescence_ior = 1.3f;
    ci::vec3 attenuation{ 1.f, 1.f, 1.f };
    float attenuation_distance = 1.f;
    float iridescence_thickness = 0.f;
    float iridescence_thickness_max = -1.f;
    float dispersion = 0.f;
    float bsdf_type_override = 0.f;
    float meta = 0.f;
    ci::vec3 conductor_k{ 0.f, 0.f, 0.f };
};

// Texture slot — resolves to either an embedded blob (copied out of the ufbx
// scene before it is freed) or an external file path relative to the FBX.
struct TextureSlot {
    bool used = false;
    std::string sourcePath;               // filename as referenced by the FBX
    std::vector<uint8_t> embedded;        // non-empty = embedded content
};

struct ImportMaterial {
    std::string name;
    std::string shadingModel;             // ufbx shading model for traceability
    EngineMaterialData data;

    TextureSlot albedoTex;
    TextureSlot normalTex;
    TextureSlot emissiveTex;
    TextureSlot roughnessTex;
    TextureSlot metallicTex;
    TextureSlot opacityTex;
};

struct ImportNode {
    std::string name;
    int parent = -1;                       // index into nodes, -1 = root
    ci::vec3 translation{ 0.f, 0.f, 0.f };
    ci::quat rotation;                     // identity
    ci::vec3 scale{ 1.f, 1.f, 1.f };
    bool retained = true;                  // false = pruned (no retained descendant)
};

struct ImportCamera {
    std::string name;
    ci::vec3 eye{ 0.f, 0.f, 0.f };
    ci::quat orient;                       // engine convention: identity looks down -Z
    float fovDeg = 45.f;                   // vertical FOV
    float nearZ = 0.1f;
    float farZ = 100.f;
    int nodeIndex = -1;
};

struct ImportLight {
    std::string name;
    std::string type;                      // point|spot|directional|area|volume
    ci::vec3 position{ 0.f, 0.f, 0.f };
    ci::vec3 direction{ 0.f, 0.f, -1.f };  // world-space aim direction
    ci::vec3 color{ 1.f, 1.f, 1.f };
    float intensity = 1.f;
    float innerAngleDeg = 0.f;             // spot only
    float outerAngleDeg = 45.f;            // spot only
    int nodeIndex = -1;
};

struct ImportScene {
    std::string sourceFile;
    bool normalizedToYUpMeters = false;
    float sourceUnitsPerCm = 0.f;          // raw FBX unit scale (0 = unknown)
    std::string warnings;                  // accumulated ufbx warnings

    std::vector<ImportMesh> meshes;
    std::vector<ImportMaterial> materials;
    std::vector<ImportNode> nodes;         // pruned to retained entities only
    std::vector<ImportCamera> cameras;
    std::vector<ImportLight> lights;
};

struct LoadOptions {
    bool normalizeUnits = true;            // Y-up right-handed, meters, camera -Z
};

// Loads and filters an FBX: keeps mesh/camera/light nodes, drops skeletons,
// joints, blend shapes and constraints. Returns nullptr-shaped scene with
// `ok=false` on failure (error filled).
struct LoadResult {
    bool ok = false;
    std::string error;
    ImportScene scene;
};
LoadResult loadFbxScene(const fs::path& fbxPath, const LoadOptions& opts);

// Helpers shared with the material mapper
ci::mat4 ufbxMatrixToCi(const void* ufbxMatrix);  // defined in FbxImporter.cpp

} // namespace fbximp
