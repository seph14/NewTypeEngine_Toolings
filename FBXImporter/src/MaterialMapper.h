#pragma once

#include "FbxImporter.h"

struct ufbx_material;

namespace fbximp {

// Maps one ufbx material onto the engine MaterialData JSON schema.
// Strategy (see docs/plan): legacy Lambert/Phong via fbx.* maps, real PBR via
// pbr.* maps; continuous transparency/transmission maps to Dielectric,
// binary opacity masks stay on Diffuse/Plastic with alphacut.
ImportMaterial mapMaterial(const ufbx_material& mat, const std::string& fallbackName);

} // namespace fbximp
