#include "OutputWriter.h"

#include "cinder/ImageIo.h"
#include "cinder/Log.h"
#include "cinder/Surface.h"
#include "cinder/TriMesh.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <map>
#include <set>

namespace fbximp {

namespace fs = std::filesystem;

//==============================================================================
// Engine-schema material JSON
//==============================================================================

ci::Json engineMaterialDataToJson(const EngineMaterialData& d) {
    ci::Json j;
    j["type"] = static_cast<uint32_t>(d.type);
    j["albedo"] = std::vector<float>{ d.albedo.x, d.albedo.y, d.albedo.z, d.albedo.w };
    j["emission"] = std::vector<float>{ d.emission.x, d.emission.y, d.emission.z };
    j["roughness"] = d.roughness;
    j["metallic"] = d.metallic;
    j["ior"] = d.ior;
    j["alphacut"] = d.alphacut;
    j["specular_tint"] = d.specular_tint;
    j["specular_trans"] = d.specular_trans;
    j["clearcoat"] = d.clearcoat;
    j["clearcoat_gloss"] = d.clearcoat_gloss;
    j["sheen"] = d.sheen;
    j["sheen_tint"] = d.sheen_tint;
    j["anisotropic"] = d.anisotropic;
    j["anisotropic_rot"] = d.anisotropic_rot;
    j["flatness"] = d.flatness;
    j["diffuse_trans"] = d.diffuse_trans;
    j["fabric"] = d.fabric;
    j["iridescence"] = d.iridescence;
    j["iridescence_ior"] = d.iridescence_ior;
    j["attenuation"] = std::vector<float>{ d.attenuation.x, d.attenuation.y, d.attenuation.z };
    j["attenuation_distance"] = d.attenuation_distance;
    j["iridescence_thickness"] = d.iridescence_thickness;
    j["iridescence_thickness_max"] = d.iridescence_thickness_max;
    j["dispersion"] = d.dispersion;
    j["bsdf_type_override"] = d.bsdf_type_override;
    j["meta"] = d.meta;
    j["k"] = std::vector<float>{ d.conductor_k.x, d.conductor_k.y, d.conductor_k.z };
    return j;
}

namespace {

std::string sanitizeName(const std::string& s) {
    std::string out;
    for (char c : s) {
        if (std::isalnum((unsigned char)c) || c == '_' || c == '-') out += c;
        else out += '_';
    }
    if (out.empty()) out = "unnamed";
    return out;
}

//==============================================================================
// Geometry writers
//==============================================================================

bool writeTriMeshBinary(const fs::path& path, const ProcessedGeometry& g) {
    ci::TriMesh::Format fmt = ci::TriMesh::Format().positions();
    if (g.normals.size() == g.positions.size()) fmt.normals();
    if (g.uvs.size() == g.positions.size()) fmt.texCoords(2);
    if (g.tangents.size() == g.positions.size()) fmt.tangents();
    if (g.bitangents.size() == g.positions.size()) fmt.bitangents();

    ci::TriMesh mesh(fmt);
    mesh.appendPositions(g.positions.data(), g.positions.size());
    if (g.normals.size() == g.positions.size())
        mesh.appendNormals(g.normals.data(), g.normals.size());
    if (g.uvs.size() == g.positions.size())
        mesh.appendTexCoords0(g.uvs.data(), g.uvs.size());
    if (g.tangents.size() == g.positions.size())
        mesh.appendTangents(g.tangents.data(), g.tangents.size());
    if (g.bitangents.size() == g.positions.size())
        mesh.appendBitangents(g.bitangents.data(), g.bitangents.size());
    mesh.appendIndices(g.indices.data(), (uint32_t)g.indices.size());

    auto target = ci::writeFile(path, /*createParents=*/true);
    if (!target) return false;
    mesh.write(target);   // v2 binary: indices + all allocated attribs
    return true;
}

bool writeObj(const fs::path& path, const ProcessedGeometry& g) {
    FILE* f = nullptr;
    if (fopen_s(&f, path.string().c_str(), "wb") != 0 || !f) return false;
    std::fprintf(f, "# NewTypeEngine FBXImporter export '%s'\n", g.name.c_str());
    std::fprintf(f, "# %zu vertices, %zu triangles\n", g.positions.size(), g.indices.size() / 3);

    char buf[256];
    for (const auto& p : g.positions) {
        std::snprintf(buf, sizeof(buf), "v %.6f %.6f %.6f\n", p.x, p.y, p.z);
        std::fputs(buf, f);
    }
    const bool hasUv = g.uvs.size() == g.positions.size();
    if (hasUv)
        for (const auto& t : g.uvs) {
            std::snprintf(buf, sizeof(buf), "vt %.6f %.6f\n", t.x, t.y);
            std::fputs(buf, f);
        }
    const bool hasNormal = g.normals.size() == g.positions.size();
    if (hasNormal)
        for (const auto& n : g.normals) {
            std::snprintf(buf, sizeof(buf), "vn %.6f %.6f %.6f\n", n.x, n.y, n.z);
            std::fputs(buf, f);
        }
    for (size_t t = 0; t + 2 < g.indices.size(); t += 3) {
        uint32_t a = g.indices[t] + 1, b = g.indices[t + 1] + 1, c = g.indices[t + 2] + 1;
        if (hasUv && hasNormal) {
            std::snprintf(buf, sizeof(buf), "f %u/%u/%u %u/%u/%u %u/%u/%u\n",
                a, a, a, b, b, b, c, c, c);
        } else if (hasNormal) {
            std::snprintf(buf, sizeof(buf), "f %u//%u %u//%u %u//%u\n", a, a, b, b, c, c);
        } else {
            std::snprintf(buf, sizeof(buf), "f %u %u %u\n", a, b, c);
        }
        std::fputs(buf, f);
    }
    std::fclose(f);
    return true;
}

//==============================================================================
// Texture extraction
//==============================================================================

// Resolve a texture slot to an image file on disk. Embedded content is
// materialized next to the output; external files are resolved relative to
// the source FBX. Returns an empty path on failure.
fs::path materializeTexture(const TextureSlot& slot, const fs::path& fbxDir,
                            const fs::path& scratchDir, const std::string& hintName) {
    if (slot.embedded.empty()) {
        if (slot.sourcePath.empty()) return {};
        fs::path src = slot.sourcePath;
        if (src.is_relative()) src = fbxDir / src;
        std::error_code ec;
        if (fs::exists(src, ec)) return src;
        CI_LOG_W("Texture file not found: '" << slot.sourcePath << "'");
        return {};
    }
    std::string ext = fs::path(slot.sourcePath).extension().string();
    if (ext.empty()) ext = ".png";
    fs::path dst = scratchDir / (hintName + ext);
    std::error_code ec;
    fs::create_directories(scratchDir, ec);
    FILE* f = nullptr;
    if (fopen_s(&f, dst.string().c_str(), "wb") != 0 || !f) return {};
    fwrite(slot.embedded.data(), 1, slot.embedded.size(), f);
    fclose(f);
    return dst;
}

ci::Surface8uRef tryLoadImage(const fs::path& path) {
    if (path.empty()) return nullptr;
    try {
        return std::make_shared<ci::Surface8u>(ci::loadImage(path));
    } catch (const std::exception& e) {
        CI_LOG_W("Failed to load image '" << path.string() << "': " << e.what());
        return nullptr;
    }
}

void copyOrWritePng(const fs::path& src, const fs::path& dst) {
    std::error_code ec;
    if (src != dst) {
        fs::copy_file(src, dst, fs::copy_options::overwrite_existing, ec);
        if (ec) CI_LOG_W("Failed to copy texture '" << src.string() << "' -> '" << dst.string() << "'");
    }
}

// Compose roughness/metallic maps into the engine's _rma convention
// (R=roughness, G=metallic, B=AO=1). Nearest-sampled to the first available
// map's resolution; scalar defaults fill missing channels.
void writeRmaTexture(const fs::path& dst, const ImportMaterial& mat,
                     const ci::Surface8uRef& rough, const ci::Surface8uRef& metal) {
    int w = 0, h = 0;
    if (rough) { w = rough->getWidth(); h = rough->getHeight(); }
    else if (metal) { w = metal->getWidth(); h = metal->getHeight(); }
    if (w <= 0 || h <= 0) return;

    ci::Surface8u out(w, h, true);
    const uint8_t metalDefault = (uint8_t)std::clamp(mat.data.metallic * 255.f, 0.f, 255.f);
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            uint8_t r = 128, g = metalDefault, b = 255;
            if (rough) {
                ci::Color8u c = rough->getPixel(ci::ivec2(
                    std::min(x, rough->getWidth() - 1), std::min(y, rough->getHeight() - 1)));
                r = c.r;
            }
            if (metal) {
                ci::Color8u c = metal->getPixel(ci::ivec2(
                    std::min(x, metal->getWidth() - 1), std::min(y, metal->getHeight() - 1)));
                g = c.r;
            }
            out.setPixel(ci::ivec2(x, y), ci::ColorA8u(r, g, b, 255));
        }
    }
    try {
        ci::writeImage(dst, out);
    } catch (const std::exception& e) {
        CI_LOG_W("Failed to write RMA texture '" << dst.string() << "': " << e.what());
    }
}

} // namespace

//==============================================================================
// writeOutputs
//==============================================================================

OutputResult writeOutputs(const fs::path& outDir, const std::string& stem,
                          const ImportScene& scene, const ProcessedScene& processed,
                          bool hierarchyMode, const OutputOptions& opts) {
    OutputResult result;
    std::error_code ec;
    fs::path meshDir = outDir / "mesh";
    fs::create_directories(meshDir, ec);
    fs::create_directories(outDir / "tex_src", ec);
    const fs::path fbxDir = fs::path(scene.sourceFile).parent_path();

    //--------------------------------------------------------------------
    // Geometries
    //--------------------------------------------------------------------
    ci::Json geometries = ci::Json::array();
    std::map<std::string, int> usedNames;    // dedupe file names
    for (const ProcessedGeometry& g : processed.geometries) {
        std::string base = stem + "_" + sanitizeName(g.name);
        int dup = ++usedNames[base];
        if (dup > 1) base += "_" + std::to_string(dup);

        fs::path file = meshDir / (base + (opts.format == GeoFormat::TriMesh ? ".msh" : ".obj"));
        bool ok = opts.format == GeoFormat::TriMesh
            ? writeTriMeshBinary(file, g)
            : writeObj(file, g);
        if (!ok) {
            result.error = "Failed writing geometry: " + file.string();
            CI_LOG_E(result.error);
            return result;
        }
        result.geometryFiles.push_back(file);

        ci::Json gj;
        gj["name"] = base;
        gj["file"] = "mesh/" + file.filename().string();
        gj["format"] = opts.format == GeoFormat::TriMesh ? "trimesh" : "obj";
        gj["material"] = scene.materials[g.materialIndex].name;
        gj["vertices"] = g.positions.size();
        gj["triangles"] = g.indices.size() / 3;
        geometries.push_back(gj);
    }

    //--------------------------------------------------------------------
    // Textures (engine naming convention, next to the JSON)
    //--------------------------------------------------------------------
    ci::Json materials = ci::Json::array();
    std::set<std::string> materialNames;
    for (const ImportMaterial& mat : scene.materials) {
        std::string safeMat = sanitizeName(mat.name);
        while (materialNames.count(safeMat)) safeMat += "_";
        materialNames.insert(safeMat);

        ci::Json mj;
        mj["name"] = mat.name;
        mj["data"] = engineMaterialDataToJson(mat.data);

        ci::Json fj;    // provenance block (not read by the engine)
        if (!mat.albedoTex.sourcePath.empty()) fj["albedo"] = mat.albedoTex.sourcePath;
        if (!mat.normalTex.sourcePath.empty()) fj["normal"] = mat.normalTex.sourcePath;
        if (!mat.emissiveTex.sourcePath.empty()) fj["emissive"] = mat.emissiveTex.sourcePath;
        if (!mat.roughnessTex.sourcePath.empty()) fj["roughness"] = mat.roughnessTex.sourcePath;
        if (!mat.metallicTex.sourcePath.empty()) fj["metallic"] = mat.metallicTex.sourcePath;
        if (!mat.opacityTex.sourcePath.empty()) fj["opacity"] = mat.opacityTex.sourcePath;
        if (fj.size() > 0) mj["fbx"] = fj;
        materials.push_back(mj);

        if (!opts.extractTextures) continue;

        // albedo (with opacity merged into alpha when present)
        ci::Surface8uRef albedoImg;
        if (mat.albedoTex.used) {
            fs::path src = materializeTexture(mat.albedoTex, fbxDir, outDir / "tex_src",
                safeMat + "_albedo_src");
            albedoImg = tryLoadImage(src);
            ci::Surface8uRef opacityImg;
            if (mat.opacityTex.used) {
                fs::path opSrc = materializeTexture(mat.opacityTex, fbxDir, outDir / "tex_src",
                    safeMat + "_opacity_src");
                opacityImg = tryLoadImage(opSrc);
            }
            if (albedoImg && opacityImg) {
                // merge opacity into alpha (engine samples albedo alpha for cutout)
                ci::Surface8u merged(albedoImg->getWidth(), albedoImg->getHeight(), true);
                for (int y = 0; y < merged.getHeight(); y++) {
                    for (int x = 0; x < merged.getWidth(); x++) {
                        ci::Color8u c = albedoImg->getPixel(ci::ivec2(x, y));
                        ci::Color8u o = opacityImg->getPixel(ci::ivec2(
                            std::min(x, opacityImg->getWidth() - 1),
                            std::min(y, opacityImg->getHeight() - 1)));
                        merged.setPixel(ci::ivec2(x, y),
                            ci::ColorA8u(c.r, c.g, c.b, o.r));
                    }
                }
                fs::path dst = outDir / (safeMat + "_albedo.png");
                try {
                    ci::writeImage(dst, merged);
                    result.textureFiles.push_back(dst);
                } catch (const std::exception& e) {
                    CI_LOG_W("Failed writing merged albedo: " << e.what());
                }
            } else if (!src.empty()) {
                fs::path dst = outDir / (safeMat + "_albedo" + src.extension().string());
                copyOrWritePng(src, dst);
                result.textureFiles.push_back(dst);
            }
        } else if (mat.opacityTex.used) {
            // opacity-only: engine reads alpha from the albedo texture
            fs::path src = materializeTexture(mat.opacityTex, fbxDir, outDir / "tex_src",
                safeMat + "_opacity_src");
            if (!src.empty()) {
                fs::path dst = outDir / (safeMat + "_albedo" + src.extension().string());
                copyOrWritePng(src, dst);
                result.textureFiles.push_back(dst);
            }
        }

        auto simpleCopy = [&](const TextureSlot& slot, const char* role) {
            if (!slot.used) return;
            fs::path src = materializeTexture(slot, fbxDir, outDir / "tex_src",
                safeMat + std::string("_") + role + "_src");
            if (src.empty()) return;
            fs::path dst = outDir / (safeMat + "_" + role + src.extension().string());
            copyOrWritePng(src, dst);
            result.textureFiles.push_back(dst);
        };
        simpleCopy(mat.normalTex, "normal");
        simpleCopy(mat.emissiveTex, "emissive");

        if (mat.roughnessTex.used || mat.metallicTex.used) {
            fs::path roughSrc = materializeTexture(mat.roughnessTex, fbxDir, outDir / "tex_src",
                safeMat + "_roughness_src");
            fs::path metalSrc = materializeTexture(mat.metallicTex, fbxDir, outDir / "tex_src",
                safeMat + "_metallic_src");
            ci::Surface8uRef rough = tryLoadImage(roughSrc);
            ci::Surface8uRef metal = tryLoadImage(metalSrc);
            if (rough || metal) {
                fs::path dst = outDir / (safeMat + "_rma.png");
                writeRmaTexture(dst, mat, rough, metal);
                result.textureFiles.push_back(dst);
            }
        }
    }

    //--------------------------------------------------------------------
    // Scene JSON
    //--------------------------------------------------------------------
    ci::Json root;
    root["format"] = "newtype.model";
    root["version"] = 1;
    ci::Json source;
    source["file"] = fs::path(scene.sourceFile).filename().string();
    source["normalized"] = scene.normalizedToYUpMeters;
    if (!scene.warnings.empty()) source["warnings"] = scene.warnings;
    root["source"] = source;
    root["materials"] = materials;
    root["geometries"] = geometries;

    if (hierarchyMode) {
        ci::Json nodes = ci::Json::array();
        for (const ImportNode& n : processed.nodes) {
            ci::Json nj;
            nj["name"] = n.name;
            nj["parent"] = n.parent;
            nj["translation"] = std::vector<float>{ n.translation.x, n.translation.y, n.translation.z };
            nj["rotation"] = std::vector<float>{ n.rotation.x, n.rotation.y, n.rotation.z, n.rotation.w };
            nj["scale"] = std::vector<float>{ n.scale.x, n.scale.y, n.scale.z };
            nj["geometries"] = ci::Json::array();   // filled below
            nodes.push_back(nj);
        }
        // attach geometry indices to their nodes
        for (size_t gi = 0; gi < processed.geometries.size(); gi++) {
            int nodeIdx = processed.geometries[gi].nodeIndex;
            if (nodeIdx >= 0 && nodeIdx < (int)nodes.size())
                nodes[nodeIdx]["geometries"].push_back((int)gi);
        }
        root["nodes"] = nodes;
    }

    ci::Json cameras = ci::Json::array();
    for (const ImportCamera& c : scene.cameras) {
        ci::Json cj;
        cj["name"] = c.name;
        cj["eye"] = std::vector<float>{ c.eye.x, c.eye.y, c.eye.z };
        cj["orient"] = std::vector<float>{ c.orient.x, c.orient.y, c.orient.z, c.orient.w };
        cj["fov"] = c.fovDeg;
        cj["near"] = c.nearZ;
        cj["far"] = c.farZ;
        cameras.push_back(cj);
    }
    root["cameras"] = cameras;

    ci::Json lights = ci::Json::array();
    for (const ImportLight& l : scene.lights) {
        ci::Json lj;
        lj["name"] = l.name;
        lj["type"] = l.type;
        lj["position"] = std::vector<float>{ l.position.x, l.position.y, l.position.z };
        lj["direction"] = std::vector<float>{ l.direction.x, l.direction.y, l.direction.z };
        lj["color"] = std::vector<float>{ l.color.x, l.color.y, l.color.z };
        lj["intensity"] = l.intensity;
        if (l.type == "spot") {
            lj["inner_angle"] = l.innerAngleDeg;
            lj["outer_angle"] = l.outerAngleDeg;
        }
        lights.push_back(lj);
    }
    root["lights"] = lights;

    result.jsonPath = outDir / (stem + ".json");
    try {
        ci::writeJson(result.jsonPath, root);
    } catch (const std::exception& e) {
        result.error = std::string("Failed writing JSON: ") + e.what();
        CI_LOG_E(result.error);
        return result;
    }

    CI_LOG_I("Wrote " << result.geometryFiles.size() << " geometry file(s), "
        << result.textureFiles.size() << " texture(s) and '" << result.jsonPath.string() << "'");
    result.ok = true;
    return result;
}

} // namespace fbximp
