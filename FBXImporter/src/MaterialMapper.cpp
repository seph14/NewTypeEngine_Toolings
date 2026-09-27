#include "MaterialMapper.h"

#include "cinder/Log.h"

#include "ufbx.h"

#include <algorithm>
#include <cmath>

namespace fbximp {

namespace {

//------------------------------------------------------------------------------
// ufbx map access helpers
//------------------------------------------------------------------------------

float mapScalar(const ufbx_material_map& map, float fallback) {
    if (!map.has_value) return fallback;
    return static_cast<float>(map.value_real);
}

ci::vec3 mapColor(const ufbx_material_map& color, const ufbx_material_map& factor) {
    ci::vec3 c{ 1.f, 1.f, 1.f };
    if (color.has_value && color.value_components >= 3) {
        c = ci::vec3{ (float)color.value_vec3.x, (float)color.value_vec3.y, (float)color.value_vec3.z };
    }
    float f = factor.has_value ? (float)factor.value_real : 1.f;
    return c * f;
}

ci::vec3 mapColor(const ufbx_material_map& color) {
    if (color.has_value && color.value_components >= 3) {
        return ci::vec3{ (float)color.value_vec3.x, (float)color.value_vec3.y, (float)color.value_vec3.z };
    }
    if (color.has_value) return ci::vec3{ (float)color.value_real };
    return ci::vec3{ 1.f, 1.f, 1.f };
}

// Legacy Blinn-Phong shininess [2..1e4] -> GGX-matched roughness.
float shininessToRoughness(double shininess) {
    if (shininess <= 0.0) return 0.5f;                       // unspecified
    float r = std::sqrt(2.f / ((float)shininess + 2.f));
    return std::clamp(r, 0.05f, 1.f);
}

//------------------------------------------------------------------------------
// Texture slot extraction — resolve the first underlying file texture
// (unwraping shader/layered wrappers) and snapshot its path/embedded bytes
// before the ufbx scene is freed.
//------------------------------------------------------------------------------

void fillSlot(TextureSlot& slot, const ufbx_material_map& map) {
    if (!map.texture || !map.texture_enabled) return;
    // unwrap shader-node / layered wrappers to the first real file texture
    const ufbx_texture* tex = map.texture;
    if (tex->type != UFBX_TEXTURE_FILE && tex->file_textures.count > 0)
        tex = tex->file_textures.data[0];
    if (!tex || tex->type != UFBX_TEXTURE_FILE) return;

    slot.used = true;
    slot.sourcePath = tex->filename.data ? tex->filename.data : "";
    if (tex->content.size > 0 && tex->content.data) {
        slot.embedded.assign(
            static_cast<const uint8_t*>(tex->content.data),
            static_cast<const uint8_t*>(tex->content.data) + tex->content.size);
    }
}

} // namespace

//==============================================================================
// mapMaterial
//==============================================================================

ImportMaterial mapMaterial(const ufbx_material& mat, const std::string& fallbackName) {
    ImportMaterial out;
    out.name = mat.name.data && mat.name.length > 0 ? mat.name.data : fallbackName;
    out.shadingModel = mat.shading_model_name.data ? mat.shading_model_name.data : "unknown";

    EngineMaterialData d;

    const bool pbr = mat.features.pbr.enabled || mat.features.metalness.enabled;
    const float metalness = pbr ? mapScalar(mat.pbr.metalness, 0.f) : 0.f;
    const float roughnessPbr = mapScalar(mat.pbr.roughness, 0.5f);
    const double shininess = mat.fbx.specular_exponent.value_real;

    // Base color / albedo
    ci::vec3 albedo = pbr
        ? mapColor(mat.pbr.base_color, mat.pbr.base_factor)
        : mapColor(mat.fbx.diffuse_color, mat.fbx.diffuse_factor);
    albedo = ci::clamp(albedo, ci::vec3(0.f), ci::vec3(1.f));

    // Emission
    ci::vec3 emission = pbr
        ? mapColor(mat.pbr.emission_color, mat.pbr.emission_factor)
        : mapColor(mat.fbx.emission_color, mat.fbx.emission_factor);
    emission = glm::max(emission, ci::vec3(0.f));

    // Transparency / transmission. Continuous -> Dielectric (approved plan).
    const float transmission = mat.features.transmission.enabled
        ? mapScalar(mat.pbr.transmission_factor, 0.f)
        : mapScalar(mat.fbx.transparency_factor, 0.f);
    const float opacity = mat.features.opacity.enabled
        ? mapScalar(mat.pbr.opacity, 1.f)
        : 1.f - std::clamp(mapScalar(mat.fbx.transparency_factor, 0.f), 0.f, 1.f); // legacy alias

    const bool isMetal = pbr && metalness >= 0.5f;
    const bool transparent = transmission > 0.f;

    if (transparent) {
        // --- Dielectric (glass) -------------------------------------------
        d.type = EngineMaterialType::Dielectric;
        d.specular_trans = 1.f;
        d.ior = mat.features.ior.enabled ? mapScalar(mat.pbr.specular_ior, 1.5f) : 1.5f;
        // Beer's-law tint: prefer the dedicated transmission color, else albedo
        ci::vec3 tint = mat.features.transmission.enabled && mat.pbr.transmission_color.has_value
            ? mapColor(mat.pbr.transmission_color)
            : albedo;
        d.attenuation = ci::clamp(tint, ci::vec3(0.01f), ci::vec3(1.f));
        float depth = mapScalar(mat.pbr.transmission_depth, 0.f);
        d.attenuation_distance = depth > 0.f ? depth : 1.f;
        d.roughness = pbr ? std::clamp(roughnessPbr, 0.f, 1.f) : shininessToRoughness(shininess);
        d.dispersion = mapScalar(mat.pbr.transmission_dispersion, 0.f);
    } else if (isMetal) {
        // --- Conductor ------------------------------------------------------
        // k stays 0 (edge-tint legacy); the engine migrates k==0 conductors to
        // Aluminium on JSON load. Albedo is kept for reference/hand-tuning.
        d.type = EngineMaterialType::Conductor;
        d.albedo = ci::vec4(albedo, 1.f);
        d.roughness = std::clamp(roughnessPbr, 0.02f, 1.f);
        d.metallic = 1.f;
    } else if (pbr) {
        // --- PBR dielectric base -> Diffuse ---------------------------------
        d.type = EngineMaterialType::Diffuse;
        d.albedo = ci::vec4(albedo, std::clamp(opacity, 0.f, 1.f));
        d.roughness = std::clamp(roughnessPbr, 0.05f, 1.f);
    } else {
        // --- Legacy Lambert / Phong -----------------------------------------
        bool isPhong = mat.shader_type == UFBX_SHADER_FBX_PHONG
            || mat.shader_type == UFBX_SHADER_BLENDER_PHONG
            || mat.shader_type == UFBX_SHADER_WAVEFRONT_MTL;
        float specFactor = mapScalar(mat.fbx.specular_factor, 0.f);
        if (isPhong && specFactor > 0.05f) {
            d.type = EngineMaterialType::Plastic;
            d.roughness = shininessToRoughness(shininess);
            // Approximate tint strength from the specular color's saturation
            ci::vec3 spec = mapColor(mat.fbx.specular_color);
            float lum = dot(spec, ci::vec3(0.2126f, 0.7152f, 0.0722f));
            ci::vec3 sat = spec - ci::vec3(lum);
            d.specular_tint = std::clamp(length(sat) * specFactor, 0.f, 1.f);
        } else {
            d.type = EngineMaterialType::Diffuse;
            d.roughness = 1.f;   // Lambertian
        }
        d.albedo = ci::vec4(albedo, std::clamp(opacity, 0.f, 1.f));
    }

    // Emission rides on whatever base type was chosen — the engine supports
    // emission on any material and its light sampler picks it up.
    d.emission = emission;

    // Binary opacity masks -> alpha cutout (fences/foliage). A scalar-only
    // continuous opacity stays in albedo.w; an actual mask texture is
    // near-binary in practice, so arm the cutout threshold.
    if (!transparent && mat.features.opacity.enabled && mat.pbr.opacity.texture)
        d.alphacut = 0.5f;

    out.data = d;

    // Texture slots (roles the engine's MaterialTextureLoader scans by name)
    fillSlot(out.albedoTex, pbr ? mat.pbr.base_color : mat.fbx.diffuse_color);
    fillSlot(out.normalTex, mat.fbx.normal_map.texture ? mat.fbx.normal_map : mat.pbr.normal_map);
    fillSlot(out.emissiveTex, pbr ? mat.pbr.emission_color : mat.fbx.emission_color);
    if (pbr) {
        fillSlot(out.roughnessTex, mat.pbr.roughness);
        fillSlot(out.metallicTex, mat.pbr.metalness);
        if (!out.albedoTex.used) fillSlot(out.albedoTex, mat.pbr.opacity); // rare: opacity as base
        else fillSlot(out.opacityTex, mat.pbr.opacity);
    } else {
        // Legacy FBX has no roughness/metallic maps; Blender PBR-recovery
        // (use_blender_pbr_material) fills them into the pbr maps.
        fillSlot(out.roughnessTex, mat.pbr.roughness);
        fillSlot(out.metallicTex, mat.pbr.metalness);
    }

    CI_LOG_I("Material '" << out.name << "' [" << out.shadingModel << "] -> engine type "
        << (uint32_t)d.type << (transparent ? " (dielectric/transmission)" : "")
        << (d.alphacut > 0.f ? " (alpha cutout)" : ""));
    return out;
}

} // namespace fbximp
