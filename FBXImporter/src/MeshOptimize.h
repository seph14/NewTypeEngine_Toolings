#pragma once

#include "GeometryProcessor.h"

namespace fbximp {

struct OptimizeOptions {
    bool optimize = true;         // lossless dedup + reordering
    float simplifyRatio = 0.f;    // (0,1) target triangle ratio; <=0 = off
    float targetError = 1e-2f;    // simplification error budget
};

// meshoptimizer pass over one geometry:
//   exact-attribute vertex dedup -> optional lossy simplify ->
//   vertex-cache reorder -> spatial (Morton) reorder -> vertex-fetch compaction.
// With optimize=false and simplifyRatio<=0 the geometry is returned unchanged.
void optimizeGeometry(ProcessedGeometry& g, const OptimizeOptions& opts);

} // namespace fbximp
