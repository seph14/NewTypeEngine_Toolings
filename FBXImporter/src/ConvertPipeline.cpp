#include "ConvertPipeline.h"

#include "cinder/Log.h"
#include "cinder/Timer.h"

namespace fbximp {

ConvertResult convertFbx(const ConvertOptions& opts) {
    ConvertResult result;
    ci::Timer timer;
    timer.start();

    if (opts.inputFbx.empty() || !fs::exists(opts.inputFbx)) {
        result.error = "Input FBX not found: " + opts.inputFbx.string();
        CI_LOG_E(result.error);
        return result;
    }

    const std::string stem = fs::path(opts.inputFbx).stem().string();
    result.outDir = opts.outDir.empty()
        ? fs::path(opts.inputFbx).parent_path() / (stem + "_imported")
        : opts.outDir;

    CI_LOG_I("=== Converting '" << opts.inputFbx.string() << "' -> '" << result.outDir.string()
        << "' (" << (opts.keepHierarchy ? "hierarchy" : "merge-by-material")
        << ", " << (opts.formatTriMesh ? "trimesh" : "obj")
        << ", optimize " << (opts.optimize ? "on" : "off")
        << ", simplify " << opts.simplifyRatio << ") ===");

    //--------------------------------------------------------------------
    // 1. Load + filter
    //--------------------------------------------------------------------
    LoadOptions loadOpts;
    loadOpts.normalizeUnits = opts.normalizeUnits;
    LoadResult loaded = loadFbxScene(opts.inputFbx, loadOpts);
    if (!loaded.ok) {
        result.error = loaded.error;
        return result;
    }

    //--------------------------------------------------------------------
    // 2. Geometry pass (merge / hierarchy) + optimize + tangents
    //--------------------------------------------------------------------
    ProcessedScene processed = opts.keepHierarchy
        ? mergeKeepingHierarchy(loaded.scene)
        : mergeByMaterial(loaded.scene);

    OptimizeOptions optOpts;
    optOpts.optimize = opts.optimize;
    optOpts.simplifyRatio = opts.simplifyRatio;
    for (ProcessedGeometry& g : processed.geometries) {
        optimizeGeometry(g, optOpts);
        finalizeAttributes(g);
        result.vertexCount += g.positions.size();
        result.triangleCount += g.indices.size() / 3;
    }

    result.geometryCount = processed.geometries.size();
    result.materialCount = loaded.scene.materials.size();
    result.nodeCount = processed.nodes.size();
    result.cameraCount = loaded.scene.cameras.size();
    result.lightCount = loaded.scene.lights.size();

    //--------------------------------------------------------------------
    // 3. Output
    //--------------------------------------------------------------------
    if (!opts.dryRun) {
        OutputOptions outOpts;
        outOpts.format = opts.formatTriMesh ? GeoFormat::TriMesh : GeoFormat::Obj;
        outOpts.extractTextures = opts.extractTextures;
        OutputResult written = writeOutputs(result.outDir, stem, loaded.scene,
            processed, opts.keepHierarchy, outOpts);
        if (!written.ok) {
            result.error = written.error;
            return result;
        }
        result.jsonPath = written.jsonPath;
        result.textureCount = written.textureFiles.size();
    }

    timer.stop();
    CI_LOG_I("=== Done in " << timer.getSeconds() << "s: " << result.geometryCount
        << " geometries / " << result.triangleCount << " tris / " << result.vertexCount
        << " verts, " << result.materialCount << " materials, "
        << result.cameraCount << " cameras, " << result.lightCount << " lights ===");
    result.ok = true;
    return result;
}

} // namespace fbximp
