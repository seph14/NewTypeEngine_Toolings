#include "MeshOptimize.h"

#include "cinder/Log.h"

#include "meshoptimizer.h"

#include <algorithm>

namespace fbximp {

namespace {

// Scatter a remap table (old -> new, ~0u = dropped) into a compacted array.
// meshopt_remapVertexBuffer copies dst[remap[i]] = src[i] for every source
// vertex, so the loop bound is the SOURCE count and dst holds newCount items.
template<typename T>
std::vector<T> remapArray(const std::vector<T>& src, const std::vector<unsigned>& remap,
                          size_t newCount) {
    std::vector<T> dst(newCount);
    meshopt_remapVertexBuffer(dst.data(), src.data(), src.size(), sizeof(T), remap.data());
    return dst;
}

std::vector<uint32_t> remapIndices(const std::vector<uint32_t>& indices,
                                   const std::vector<unsigned>& remap) {
    std::vector<uint32_t> dst(indices.size());
    meshopt_remapIndexBuffer(dst.data(), indices.data(), indices.size(), remap.data());
    return dst;
}

} // namespace

void optimizeGeometry(ProcessedGeometry& g, const OptimizeOptions& opts) {
    if (!opts.optimize && opts.simplifyRatio <= 0.f) return;
    if (g.indices.empty() || g.positions.empty()) return;

    const size_t trisBefore = g.indices.size() / 3;
    const size_t vertsBefore = g.positions.size();
    float simplifyError = 0.f;

    //--------------------------------------------------------------------
    // 1. Exact-attribute vertex dedup (collapses the corner expansion)
    //--------------------------------------------------------------------
    {
        meshopt_Stream streams[3];
        size_t streamCount = 0;
        // meshopt_Stream::size is the per-vertex byte count, stride the byte
        // distance between vertices (both 12/8 here — tightly packed arrays).
        streams[streamCount++] = { g.positions.data(), sizeof(ci::vec3), sizeof(ci::vec3) };
        if (g.normals.size() == g.positions.size())
            streams[streamCount++] = { g.normals.data(), sizeof(ci::vec3), sizeof(ci::vec3) };
        if (g.uvs.size() == g.positions.size())
            streams[streamCount++] = { g.uvs.data(), sizeof(ci::vec2), sizeof(ci::vec2) };

        std::vector<unsigned> remap(g.positions.size());
        size_t unique = meshopt_generateVertexRemapMulti(
            remap.data(), g.indices.data(), g.indices.size(), g.positions.size(),
            streams, streamCount);

        g.positions = remapArray(g.positions, remap, unique);
        if (!g.normals.empty()) g.normals = remapArray(g.normals, remap, unique);
        if (!g.uvs.empty()) g.uvs = remapArray(g.uvs, remap, unique);
        g.indices = remapIndices(g.indices, remap);
    }

    //--------------------------------------------------------------------
    // 2. Optional lossy simplification (position metric)
    //    The edge-collapse classifier needs a position-welded, compact mesh:
    //    attribute-split vertices read as all-border, and unreferenced
    //    duplicates lock their positions. So: weld by position only,
    //    simplify, then map indices back to representative original vertices
    //    (attribute seams survive as split corners, as before).
    //--------------------------------------------------------------------
    if (opts.simplifyRatio > 0.f && opts.simplifyRatio < 1.f && g.indices.size() >= 6) {
        size_t target = (size_t)(g.indices.size() * opts.simplifyRatio) / 3 * 3;
        target = std::max(target, size_t(3));

        meshopt_Stream posStream = { g.positions.data(), sizeof(ci::vec3), sizeof(ci::vec3) };
        std::vector<unsigned> weld(g.positions.size());
        size_t weldedCount = meshopt_generateVertexRemapMulti(
            weld.data(), g.indices.data(), g.indices.size(), g.positions.size(),
            &posStream, 1);
        std::vector<ci::vec3> weldedPos = remapArray(g.positions, weld, weldedCount);
        std::vector<uint32_t> weldedIdx = remapIndices(g.indices, weld);

        // welded -> representative original vertex (first corner at that spot)
        std::vector<uint32_t> weldedToOriginal(weldedCount, ~0u);
        for (size_t i = 0; i < g.positions.size(); i++)
            if (weldedToOriginal[weld[i]] == ~0u) weldedToOriginal[weld[i]] = (uint32_t)i;

        std::vector<uint32_t> simplified(weldedIdx.size());
        size_t newIndexCount = meshopt_simplify(
            simplified.data(), weldedIdx.data(), weldedIdx.size(),
            reinterpret_cast<const float*>(weldedPos.data()), weldedCount,
            sizeof(ci::vec3), target, opts.targetError, /*options=*/0, &simplifyError);
        simplified.resize(newIndexCount);
        for (uint32_t& idx : simplified) idx = weldedToOriginal[idx];
        g.indices = std::move(simplified);
    }

    //--------------------------------------------------------------------
    // 3. Vertex-cache reorder (cheap; relevant if data is ever rastered)
    //--------------------------------------------------------------------
    {
        std::vector<uint32_t> reordered(g.indices.size());
        meshopt_optimizeVertexCache(reordered.data(), g.indices.data(), g.indices.size(),
                                     g.positions.size());
        g.indices = std::move(reordered);
    }

    //--------------------------------------------------------------------
    // 4. Spatial (Morton) triangle order — improves BVH build locality
    //--------------------------------------------------------------------
    {
        std::vector<unsigned> remap(g.positions.size());
        meshopt_spatialSortRemap(remap.data(),
            reinterpret_cast<const float*>(g.positions.data()), g.positions.size(),
            sizeof(ci::vec3));
        g.positions = remapArray(g.positions, remap, g.positions.size());
        if (!g.normals.empty()) g.normals = remapArray(g.normals, remap, g.normals.size());
        if (!g.uvs.empty()) g.uvs = remapArray(g.uvs, remap, g.uvs.size());
        g.indices = remapIndices(g.indices, remap);
    }

    //--------------------------------------------------------------------
    // 5. Vertex-fetch compaction (also drops vertices orphaned by simplify)
    //--------------------------------------------------------------------
    {
        std::vector<unsigned> remap(g.positions.size());
        size_t unique = meshopt_optimizeVertexFetchRemap(remap.data(), g.indices.data(),
            g.indices.size(), g.positions.size());
        g.positions = remapArray(g.positions, remap, unique);
        if (!g.normals.empty()) g.normals = remapArray(g.normals, remap, unique);
        if (!g.uvs.empty()) g.uvs = remapArray(g.uvs, remap, unique);
        g.indices = remapIndices(g.indices, remap);
    }

    const size_t trisAfter = g.indices.size() / 3;
    const size_t vertsAfter = g.positions.size();
    if (opts.simplifyRatio > 0.f) {
        CI_LOG_I("Optimized '" << g.name << "': " << trisBefore << " -> " << trisAfter
            << " tris, " << vertsBefore << " -> " << vertsAfter
            << " verts (simplify error " << simplifyError << ")");
    } else {
        CI_LOG_I("Optimized '" << g.name << "': " << trisBefore << " -> " << trisAfter
            << " tris, " << vertsBefore << " -> " << vertsAfter << " verts");
    }
}

} // namespace fbximp
