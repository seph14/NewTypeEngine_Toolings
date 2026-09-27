#include "FbxImporter.h"
#include "MaterialMapper.h"

#include "cinder/Log.h"

#include "ufbx.h"

#include <algorithm>
#include <map>
#include <sstream>

namespace fbximp {

ci::mat4 ufbxMatrixToCi(const void* ufbxMatrix) {
    auto* m = static_cast<const ufbx_matrix*>(ufbxMatrix);
    // ufbx stores m{row}{col}; ci::mat4 takes column-major values.
    return ci::mat4(
        (float)m->m00, (float)m->m10, (float)m->m20, 0.f,
        (float)m->m01, (float)m->m11, (float)m->m21, 0.f,
        (float)m->m02, (float)m->m12, (float)m->m22, 0.f,
        (float)m->m03, (float)m->m13, (float)m->m23, 1.f);
}

namespace {

std::string ufbxStr(const ufbx_string& s) {
    return s.data ? std::string(s.data, s.length) : std::string();
}

ci::vec3 toVec3(const ufbx_vec3& v) { return ci::vec3{ (float)v.x, (float)v.y, (float)v.z }; }
ci::vec2 toVec2(const ufbx_vec2& v) { return ci::vec2{ (float)v.x, (float)v.y }; }

//==============================================================================
// Per-(node x material) mesh extraction with fan triangulation
//==============================================================================

struct CornerSource {
    const ufbx_vertex_vec3* position;
    const ufbx_vertex_vec3* normal;
    const ufbx_vertex_vec2* uv;
};

} // namespace

//==============================================================================
// loadFbxScene
//==============================================================================

LoadResult loadFbxScene(const fs::path& fbxPath, const LoadOptions& opts) {
    LoadResult result;

    ufbx_load_opts loadOpts{};
    // Blender exports PBR values as legacy Phong; this recovers
    // roughness/metalness (incl. textures) into the pbr maps.
    loadOpts.use_blender_pbr_material = true;
    loadOpts.clean_skin_weights = true;

    if (opts.normalizeUnits) {
        // Y-up, right-handed, +Z front (i.e. forward = -Z, matching the
        // engine/Cinder camera convention), 1 unit = 1 meter.
        loadOpts.target_axes.right = UFBX_COORDINATE_AXIS_POSITIVE_X;
        loadOpts.target_axes.up = UFBX_COORDINATE_AXIS_POSITIVE_Y;
        loadOpts.target_axes.front = UFBX_COORDINATE_AXIS_POSITIVE_Z;
        loadOpts.target_unit_meters = 1.0f;
        loadOpts.target_camera_axes = loadOpts.target_axes;
        loadOpts.target_light_axes = loadOpts.target_axes;
    }

    std::string pathStr = fbxPath.string();
    ufbx_error error;
    ufbx_scene* scene = ufbx_load_file(pathStr.c_str(), &loadOpts, &error);
    if (!scene) {
        char buf[512];
        ufbx_format_error(buf, sizeof(buf), &error);
        result.error = buf;
        CI_LOG_E("ufbx failed to load '" << pathStr << "': " << buf);
        return result;
    }

    ImportScene& out = result.scene;
    out.sourceFile = pathStr;
    out.normalizedToYUpMeters = opts.normalizeUnits;
    out.sourceUnitsPerCm = scene->settings.original_unit_meters > 0
        ? (float)scene->settings.original_unit_meters * 100.f : 0.f;

    // --- Warnings -----------------------------------------------------------
    if (scene->metadata.warnings.count > 0) {
        std::stringstream ss;
        for (size_t i = 0; i < scene->metadata.warnings.count; i++) {
            const ufbx_warning& w = scene->metadata.warnings.data[i];
            ss << ufbxStr(w.description) << "\n";
        }
        out.warnings = ss.str();
        CI_LOG_W("ufbx reported " << scene->metadata.warnings.count << " warning(s):\n" << out.warnings);
    }

    // --- Material table (per-instance materials, deduped by pointer) --------
    std::map<const ufbx_material*, int> materialIndex;
    int fallbackMaterial = -1;
    auto internMaterial = [&](const ufbx_material* mat) -> int {
        if (!mat) {
            if (fallbackMaterial < 0) {
                ImportMaterial fallback;
                fallback.name = "__default__";
                fallback.shadingModel = "fallback";
                fallback.data.type = EngineMaterialType::Diffuse;
                fallback.data.albedo = ci::vec4(0.8f, 0.8f, 0.8f, 1.f);
                fallback.data.roughness = 0.8f;
                fallbackMaterial = (int)out.materials.size();
                out.materials.push_back(fallback);
            }
            return fallbackMaterial;
        }
        auto it = materialIndex.find(mat);
        if (it != materialIndex.end()) return it->second;
        int idx = (int)out.materials.size();
        materialIndex[mat] = idx;
        out.materials.push_back(mapMaterial(*mat, "Material_" + std::to_string(idx)));
        return idx;
    };

    // --- Node table: retain nodes needed by meshes/cameras/lights ----------
    std::map<const ufbx_node*, int> nodeIndex;
    std::vector<const ufbx_node*> retained;
    // mark entity nodes + ancestors
    std::map<const ufbx_node*, bool> isRetained;
    for (size_t i = 0; i < scene->nodes.count; i++) {
        const ufbx_node* node = scene->nodes.data[i];
        bool entity = node->mesh || node->camera || node->light;
        if (!entity) continue;
        for (const ufbx_node* n = node; n; n = n->parent)
            isRetained[n] = true;
    }
    for (size_t i = 0; i < scene->nodes.count; i++) {
        const ufbx_node* node = scene->nodes.data[i];
        if (!isRetained[node]) continue;
        ImportNode n;
        n.name = ufbxStr(node->name);
        n.translation = toVec3(node->local_transform.translation);
        const ufbx_quat& q = node->local_transform.rotation;
        n.rotation = ci::quat((float)q.w, (float)q.x, (float)q.y, (float)q.z);
        n.scale = toVec3(node->local_transform.scale);
        nodeIndex[node] = (int)out.nodes.size();
        out.nodes.push_back(n);
        retained.push_back(node);
    }
    // resolve parent links
    for (size_t i = 0; i < retained.size(); i++) {
        const ufbx_node* parent = retained[i]->parent;
        out.nodes[i].parent = parent ? (nodeIndex.count(parent) ? nodeIndex[parent] : -1) : -1;
    }

    // --- Meshes / cameras / lights ------------------------------------------
    size_t totalTris = 0, totalVerts = 0;
    for (size_t i = 0; i < retained.size(); i++) {
        const ufbx_node* node = retained[i];

        if (node->mesh) {
            const ufbx_mesh& mesh = *node->mesh;
            if (!mesh.vertex_position.exists) continue;
            CornerSource src{ &mesh.vertex_position, &mesh.vertex_normal, &mesh.vertex_uv };

            // Split faces by material slot (per-instance node->materials).
            auto emitPart = [&](uint32_t slot, const ufbx_mesh& m, const CornerSource& s) {
                ImportMesh im;
                im.name = ufbxStr(node->name);
                im.nodeIndex = (int)i;
                im.worldMatrix = ufbxMatrixToCi(&node->geometry_to_world);

                const ufbx_material* mat = nullptr;
                if (slot != UFBX_NO_INDEX && slot < node->materials.count)
                    mat = node->materials.data[slot];
                im.materialIndex = internMaterial(mat);

                // face filter by slot
                const bool hasNormal = s.normal && s.normal->exists;
                const bool hasUv = s.uv && s.uv->exists;
                for (size_t f = 0; f < m.faces.count; f++) {
                    const ufbx_face& face = m.faces.data[f];
                    if (face.num_indices < 3) continue;
                    uint32_t fs = UFBX_NO_INDEX;
                    if (m.face_material.count > f) fs = m.face_material.data[f];
                    if (fs != slot) continue;

                    if (face.num_indices > 64) continue;
                    struct Corner { ci::vec3 p, n; ci::vec2 uv; } corners[64];
                    size_t n = 0; bool broken = false;
                    for (size_t c = 0; c < face.num_indices; c++) {
                        uint32_t ci2 = face.index_begin + (uint32_t)c;
                        uint32_t pi = s.position->indices.data[ci2];
                        if (pi == UFBX_NO_INDEX) { broken = true; break; }
                        Corner corner;
                        corner.p = toVec3(s.position->values.data[pi]);
                        if (hasNormal) {
                            uint32_t ni = s.normal->indices.data[ci2];
                            corner.n = ni == UFBX_NO_INDEX ? ci::vec3(0, 1, 0)
                                                           : toVec3(s.normal->values.data[ni]);
                        } else corner.n = ci::vec3(0);
                        if (hasUv) {
                            uint32_t ui = s.uv->indices.data[ci2];
                            corner.uv = ui == UFBX_NO_INDEX ? ci::vec2(0)
                                                            : toVec2(s.uv->values.data[ui]);
                        } else corner.uv = ci::vec2(0);
                        corners[n++] = corner;
                    }
                    if (broken) continue;
                    for (size_t k = 1; k + 1 < n; k++) {
                        uint32_t base = (uint32_t)im.positions.size();
                        const Corner& a = corners[0];
                        const Corner& b = corners[k];
                        const Corner& c = corners[k + 1];
                        im.positions.insert(im.positions.end(), { a.p, b.p, c.p });
                        if (hasNormal) im.normals.insert(im.normals.end(), { a.n, b.n, c.n });
                        if (hasUv) im.uvs.insert(im.uvs.end(), { a.uv, b.uv, c.uv });
                        im.indices.insert(im.indices.end(), { base, base + 1, base + 2 });
                        im.sourceTriangles++;
                    }
                }
                if (im.sourceTriangles > 0) {
                    totalTris += im.sourceTriangles;
                    totalVerts += im.positions.size();
                    out.meshes.push_back(std::move(im));
                }
            };

            if (mesh.face_material.count > 0) {
                // collect the distinct slots present in this mesh
                std::map<uint32_t, bool> slots;
                for (size_t f = 0; f < mesh.face_material.count; f++)
                    slots[mesh.face_material.data[f]] = true;
                for (auto& [slot, dummy] : slots) emitPart(slot, mesh, src);
            } else {
                emitPart(UFBX_NO_INDEX, mesh, src);
            }
        }

        if (node->camera) {
            const ufbx_camera& cam = *node->camera;
            ImportCamera c;
            c.name = ufbxStr(node->name);
            ci::mat4 world = ufbxMatrixToCi(&node->node_to_world);
            c.eye = ci::vec3(world[3]);
            c.orient = ci::quat(ci::mat3(world));
            c.fovDeg = cam.projection_mode == UFBX_PROJECTION_MODE_PERSPECTIVE
                ? std::clamp((float)cam.field_of_view_deg.y, 1.f, 175.f) : 45.f;
            c.nearZ = cam.near_plane > 0.f ? (float)cam.near_plane : 0.1f;
            c.farZ = cam.far_plane > c.nearZ ? (float)cam.far_plane : 100.f;
            c.nodeIndex = (int)i;
            out.cameras.push_back(c);
            CI_LOG_I("Camera '" << c.name << "' fov=" << c.fovDeg
                << " eye=(" << c.eye.x << ", " << c.eye.y << ", " << c.eye.z << ")");
        }

        if (node->light) {
            const ufbx_light& light = *node->light;
            ImportLight l;
            switch (light.type) {
            case UFBX_LIGHT_POINT: l.type = "point"; break;
            case UFBX_LIGHT_DIRECTIONAL: l.type = "directional"; break;
            case UFBX_LIGHT_SPOT: l.type = "spot"; break;
            case UFBX_LIGHT_AREA: l.type = "area"; break;
            default: l.type = "volume"; break;
            }
            l.name = ufbxStr(node->name);
            ufbx_vec3 pos = ufbx_transform_position(&node->node_to_world, ufbx_vec3{ 0, 0, 0 });
            l.position = toVec3(pos);
            ufbx_vec3 dir = ufbx_transform_direction(&node->node_to_world, light.local_direction);
            l.direction = normalize(toVec3(dir));
            l.color = toVec3(light.color);
            l.intensity = (float)light.intensity;
            l.innerAngleDeg = (float)light.inner_angle;
            l.outerAngleDeg = (float)light.outer_angle;
            l.nodeIndex = (int)i;
            out.lights.push_back(l);
            CI_LOG_I("Light '" << l.name << "' type=" << l.type
                << " intensity=" << l.intensity);
        }
    }

    ufbx_free_scene(scene);

    CI_LOG_I("Loaded '" << pathStr << "': " << out.meshes.size() << " mesh parts, "
        << out.materials.size() << " materials, " << out.cameras.size() << " cameras, "
        << out.lights.size() << " lights, " << out.nodes.size() << " retained nodes "
        << "(" << totalVerts << " expanded verts, " << totalTris << " tris)");

    result.ok = true;
    return result;
}

} // namespace fbximp
