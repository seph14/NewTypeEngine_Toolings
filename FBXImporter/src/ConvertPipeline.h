#pragma once

#include "FbxImporter.h"
#include "GeometryProcessor.h"
#include "MeshOptimize.h"
#include "OutputWriter.h"

namespace fbximp {

struct ConvertOptions {
    fs::path inputFbx;
    fs::path outDir;               // empty -> <fbxDir>/<stem>_imported
    bool keepHierarchy = false;
    bool optimize = true;
    float simplifyRatio = 0.f;     // 0 = off
    bool formatTriMesh = true;     // false = OBJ
    bool normalizeUnits = true;
    bool extractTextures = true;
    bool dryRun = false;           // load + process + stats, write nothing
};

struct ConvertResult {
    bool ok = false;
    std::string error;
    fs::path outDir;
    fs::path jsonPath;
    size_t geometryCount = 0;
    size_t materialCount = 0;
    size_t vertexCount = 0;
    size_t triangleCount = 0;
    size_t nodeCount = 0;
    size_t cameraCount = 0;
    size_t lightCount = 0;
    size_t textureCount = 0;
};

ConvertResult convertFbx(const ConvertOptions& opts);

} // namespace fbximp
