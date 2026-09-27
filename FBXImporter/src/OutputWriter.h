#pragma once

#include "FbxImporter.h"
#include "GeometryProcessor.h"

#include "cinder/Json.h"

namespace fbximp {

enum class GeoFormat { TriMesh, Obj };

struct OutputOptions {
    GeoFormat format = GeoFormat::TriMesh;
    bool extractTextures = true;
};

struct OutputResult {
    bool ok = false;
    std::string error;
    fs::path jsonPath;
    std::vector<fs::path> geometryFiles;
    std::vector<fs::path> textureFiles;
};

// Writes <outDir>/<stem>.json, <outDir>/mesh/<stem>_<geom>.(msh|obj) and the
// engine-convention textures next to the JSON. scene provides materials/
// cameras/lights; processed provides the final geometries (+ nodes in
// hierarchy mode).
OutputResult writeOutputs(const fs::path& outDir, const std::string& stem,
                          const ImportScene& scene, const ProcessedScene& processed,
                          bool hierarchyMode, const OutputOptions& opts);

// Engine-schema material JSON (byte-compatible with
// MaterialPool::materialDataToJson, see src/newtype/render/MaterialPool.cpp).
ci::Json engineMaterialDataToJson(const EngineMaterialData& d);

} // namespace fbximp
