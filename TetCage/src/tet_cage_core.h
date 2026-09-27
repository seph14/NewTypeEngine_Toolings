//==============================================================================
// tet_cage — Stage-1 offline tetrahedral-cage builder + Gate-1 validator
// (tools/tet_cage/tet_cage.cpp)
//
// Implements the preprocessing half of "Ray Tracing Massive Amounts of
// Animated Geometry" (Gruen et al., HPG 2026) as staged in
// docs/tetrahedral-cage-prototype.md:
//
//   rest-pose mesh -> voxel grid -> 6 conforming tets/voxel (Freudenthal)
//   -> clip every source triangle into the tets it overlaps
//   -> deterministic shared-face ownership for coplanar triangles
//   -> per-tet "pieces" -> .tetcage binary
//
// Gate-1 validation (must pass before Stage 2):
//   1. area conservation   (sum of piece areas == sum of source areas)
//   2. cross-face segment pairing (identical cuts on both sides of every
//      shared tet face -> no cracks)
//   3. two-way surface sampling (piece soup lies on the source surface and
//      vice versa)
//
// No engine/LuisaCompute dependency — build via run.bat (MSVC, C++20).
//==============================================================================

#pragma once
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <cstdarg>
#include <stdexcept>
#include <vector>

namespace fs = std::filesystem;

//==============================================================================
// Small utilities
//==============================================================================

// Fatal errors throw instead of exiting so the GUI can catch them; the CLI
// driver catches, prints, and exits 2 (identical observable behavior).
static void die(const std::string& msg) {
    throw std::runtime_error("tet_cage: error: " + msg);
}

// Log routing: CLI output goes to stdout; the GUI installs a sink to mirror
// builder/validator output into its log panel.
static std::function<void(const std::string&)> g_logSink;
static void tcl(const char* fmt, ...) {
    char buf[2048];
    va_list args;
    va_start(args, fmt);
    int n = std::vsnprintf(buf, sizeof buf, fmt, args);
    va_end(args);
    if (n <= 0) return;
    std::fwrite(buf, 1, (size_t)n, stdout);
    if (g_logSink) g_logSink(std::string(buf, (size_t)n));
}

static double nowSec() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

//==============================================================================
// Math types
//==============================================================================

struct Vec2 { float x = 0.f, y = 0.f; };

struct Vec3 {
    float x = 0.f, y = 0.f, z = 0.f;
};

static Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
static Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
static float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
static float length(Vec3 a) { return std::sqrt(dot(a, a)); }
static Vec3 normalize(Vec3 a) {
    float l = length(a);
    return l > 1e-20f ? a * (1.f / l) : Vec3{0.f, 1.f, 0.f};
}

struct Vert { Vec3 p, n; Vec2 uv; };
struct Tri { uint32_t a, b, c; };

struct Mesh {
    std::vector<Vert> verts;
    std::vector<Tri> tris;
    // diagnostics
    float totalArea() const {
        double s = 0.0;
        for (const auto& t : tris)
            s += 0.5 * (double)length(cross(verts[t.b].p - verts[t.a].p,
                                            verts[t.c].p - verts[t.a].p));
        return (float)s;
    }
};

//==============================================================================
// OBJ loader (v / vn / vt / f; fan triangulation; negative indices)
//==============================================================================

static Mesh loadOBJ(const fs::path& path) {
    std::ifstream in(path);
    if (!in) die("cannot open OBJ " + path.string());

    std::vector<Vec3> pos, nrm;
    std::vector<Vec2> uv;
    std::string line;
    size_t lineNo = 0;
    while (std::getline(in, line)) {
        ++lineNo;
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string tag;
        ss >> tag;
        if (tag == "v") {
            Vec3 p; ss >> p.x >> p.y >> p.z; pos.push_back(p);
        } else if (tag == "vn") {
            Vec3 n; ss >> n.x >> n.y >> n.z; nrm.push_back(n);
        } else if (tag == "vt") {
            Vec2 t; ss >> t.x >> t.y; uv.push_back(t);
        }
        // faces are parsed in the second pass below (they need per-corner
        // (v,vt,vn) resolution into unique output vertices)
    }
    (void)lineNo;

    if (pos.empty()) die("OBJ contains no vertices: " + path.string());

    // Re-parse faces to keep per-corner attribute indices (simplest correct path:
    // duplicate vertices per unique (v,vt,vn) combination).
    // The first pass gave us triangulated position indices; do it again fully.
    std::ifstream in2(path);
    std::unordered_map<uint64_t, uint32_t> cornerCache;
    cornerCache.reserve(pos.size() * 2);
    Mesh mesh;
    auto cornerIndex = [&](int v, int t, int n) -> uint32_t {
        uint64_t key = ((uint64_t)(uint32_t)v << 40) ^ ((uint64_t)(uint32_t)t << 20) ^
                       (uint64_t)(uint32_t)n;
        auto it = cornerCache.find(key);
        if (it != cornerCache.end()) return it->second;
        Vert vert;
        vert.p = pos[(size_t)v - 1];
        vert.uv = (t > 0 && (size_t)t - 1 < uv.size()) ? uv[(size_t)t - 1] : Vec2{};
        vert.n = (n > 0 && (size_t)n - 1 < nrm.size()) ? normalize(nrm[(size_t)n - 1]) : Vec3{};
        uint32_t idx = (uint32_t)mesh.verts.size();
        mesh.verts.push_back(vert);
        cornerCache.emplace(key, idx);
        return idx;
    };
    while (std::getline(in2, line)) {
        if (line.empty() || line[0] == '#' || line.compare(0, 2, "f ") != 0) continue;
        std::istringstream ss(line);
        std::string tag; ss >> tag;
        std::vector<uint32_t> idx;
        std::string tok;
        while (ss >> tok) {
            int v = 0, t = 0, n = 0;
            int fields[3] = {0, 0, 0};
            int fi = 0;
            size_t start = 0;
            for (size_t i = 0; i <= tok.size(); ++i) {
                if (i == tok.size() || tok[i] == '/') {
                    if (start < i) {
                        long val = std::strtol(tok.c_str() + start, nullptr, 10);
                        if (val < 0) val += (long)(fi == 0 ? pos.size() : fi == 1 ? uv.size() : nrm.size());
                        fields[fi] = (int)val;
                    }
                    ++fi;
                    start = i + 1;
                }
            }
            v = fields[0]; t = fields[1]; n = fields[2];
            if (v > 0) idx.push_back(cornerIndex(v, t, n));
        }
        for (size_t i = 1; i + 1 < idx.size(); ++i)
            mesh.tris.push_back({idx[0], idx[i], idx[i + 1]});
    }

    // If the file had no normals, compute area-weighted vertex normals.
    bool anyNormal = false;
    for (const auto& v : mesh.verts)
        if (length(v.n) > 0.5f) { anyNormal = true; break; }
    if (!anyNormal) {
        std::vector<Vec3> acc(mesh.verts.size(), Vec3{});
        for (const auto& t : mesh.tris) {
            Vec3 fn = cross(mesh.verts[t.b].p - mesh.verts[t.a].p,
                            mesh.verts[t.c].p - mesh.verts[t.a].p);
            acc[t.a] = acc[t.a] + fn;
            acc[t.b] = acc[t.b] + fn;
            acc[t.c] = acc[t.c] + fn;
        }
        for (size_t i = 0; i < mesh.verts.size(); ++i)
            mesh.verts[i].n = normalize(acc[i]);
    }
    return mesh;
}

//==============================================================================
// VAT loader (standalone re-implementation of scene/VATLoader.cpp semantics)
//==============================================================================

struct VATInfo {
    uint32_t vertexCount = 0, frameCount = 0, indexCount = 0;
};

static void readOrDie(FILE* f, void* dst, size_t bytes, const fs::path& path) {
    if (std::fread(dst, 1, bytes, f) != bytes)
        die("unexpected EOF reading " + path.string());
}

// Loads topology `topoIndex`, frame `frame` as the rest-pose mesh.
static Mesh loadVAT(const fs::path& path, int topoIndex, uint32_t frame) {
    FILE* f = std::fopen(path.string().c_str(), "rb");
    if (!f) die("cannot open VAT " + path.string());

    uint32_t version = 0;
    readOrDie(f, &version, 4, path);
    if (version != 0 && version != 1) die("unsupported VAT version " + std::to_string(version));

    // Header table
    struct TopoHeader {
        float center[3], extend[3];
        uint32_t indexCount, vertexCount, frameCount;
    };
    std::vector<TopoHeader> headers;
    if (version == 1) {
        uint32_t topoCount = 0;
        readOrDie(f, &topoCount, 4, path);
        if (topoCount == 0) die("empty topology table");
        headers.resize(topoCount);
        for (auto& h : headers) {
            readOrDie(f, h.center, 12, path);
            readOrDie(f, h.extend, 12, path);
            readOrDie(f, &h.indexCount, 4, path);
            readOrDie(f, &h.vertexCount, 4, path);
            readOrDie(f, &h.frameCount, 4, path);
        }
    }
    int topoCount = (int)headers.size();
    if (version == 0) topoCount = 1;
    if (topoIndex < 0 || topoIndex >= topoCount)
        die("topology index out of range (file has " + std::to_string(topoCount) + ")");

    // Walk to the requested topology payload
    for (int t = 0; t < topoIndex; ++t) {
        if (version == 0) {
            float cb[6];
            readOrDie(f, cb, 24, path);                       // center+extend
            uint32_t indexCount = 0, vertexCount = 0;
            readOrDie(f, &indexCount, 4, path);
            std::fseek(f, (long)indexCount * 4, SEEK_CUR);    // indices
            readOrDie(f, &vertexCount, 4, path);
            std::fseek(f, (long)vertexCount * 8, SEEK_CUR);   // texcoords
            uint32_t totalFloats = 0;
            readOrDie(f, &totalFloats, 4, path);
            uint32_t totalVerts = totalFloats / 3u;
            std::fseek(f, (long)totalVerts * 12, SEEK_CUR);   // positions
            std::fseek(f, (long)totalVerts * 12, SEEK_CUR);   // normals
        } else {
            uint32_t indexCount = headers[(size_t)t].indexCount;
            uint32_t vertexCount = headers[(size_t)t].vertexCount;
            std::fseek(f, (long)indexCount * 4, SEEK_CUR);
            std::fseek(f, (long)vertexCount * 8, SEEK_CUR);
            std::fseek(f, (long)(vertexCount * headers[(size_t)t].frameCount) * 12, SEEK_CUR);
            std::fseek(f, (long)(vertexCount * headers[(size_t)t].frameCount) * 12, SEEK_CUR);
        }
    }

    // Read the requested topology. V0 field order: center/extend, indexCount,
    // indices[], vertexCount, texcoords[], totalVertFloats, positions, normals
    // (per scene/VATLoader.cpp parse_v0).
    uint32_t indexCount = 0, vertexCount = 0, frameCount = 0;
    std::vector<uint32_t> indices;
    if (version == 0) {
        float cb[6];
        readOrDie(f, cb, 24, path);
        readOrDie(f, &indexCount, 4, path);
        indices.resize(indexCount);
        readOrDie(f, indices.data(), (size_t)indexCount * 4, path);
        readOrDie(f, &vertexCount, 4, path);
    } else {
        indexCount = headers[(size_t)topoIndex].indexCount;
        vertexCount = headers[(size_t)topoIndex].vertexCount;
        frameCount = headers[(size_t)topoIndex].frameCount;
        indices.resize(indexCount);
        readOrDie(f, indices.data(), (size_t)indexCount * 4, path);
    }
    if (vertexCount == 0) die("topology has no vertices");

    std::vector<float> texcoords((size_t)vertexCount * 2u);
    readOrDie(f, texcoords.data(), (size_t)vertexCount * 8, path);
    uint32_t totalVerts = vertexCount * frameCount;
    if (version == 0) {
        uint32_t totalFloats = 0;
        readOrDie(f, &totalFloats, 4, path);
        totalVerts = totalFloats / 3u;
        frameCount = totalVerts / vertexCount;
    }
    if (frame >= frameCount) die("frame out of range (file has " + std::to_string(frameCount) + ")");

    std::vector<float> rawPos((size_t)totalVerts * 3u);
    readOrDie(f, rawPos.data(), (size_t)totalVerts * 12, path);
    std::vector<float> rawNrm((size_t)totalVerts * 3u);
    readOrDie(f, rawNrm.data(), (size_t)totalVerts * 12, path);
    std::fclose(f);

    Mesh mesh;
    mesh.verts.resize(vertexCount);
    size_t base = (size_t)frame * vertexCount;
    for (uint32_t i = 0; i < vertexCount; ++i) {
        Vec3 p{rawPos[(base + i) * 3 + 0], rawPos[(base + i) * 3 + 1], rawPos[(base + i) * 3 + 2]};
        Vec3 n{rawNrm[(base + i) * 3 + 0], rawNrm[(base + i) * 3 + 1], rawNrm[(base + i) * 3 + 2]};
        if (std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z)) n = {0.f, 1.f, 0.f};
        mesh.verts[i].p = p;
        mesh.verts[i].n = normalize(n);
        mesh.verts[i].uv = {texcoords[(size_t)i * 2u], texcoords[(size_t)i * 2u + 1]};
    }
    mesh.tris.reserve(indexCount / 3);
    for (uint32_t i = 0; i + 2 < indexCount; i += 3)
        mesh.tris.push_back({indices[i], indices[i + 1], indices[i + 2]});
    if (mesh.tris.empty()) die("VAT topology has no triangles");
    return mesh;
}

static int vatTopologyCount(const fs::path& path) {
    FILE* f = std::fopen(path.string().c_str(), "rb");
    if (!f) die("cannot open VAT " + path.string());
    uint32_t version = 0;
    readOrDie(f, &version, 4, path);
    std::fclose(f);
    if (version == 0) return 1;
    if (version == 1) {
        f = std::fopen(path.string().c_str(), "rb");
        readOrDie(f, &version, 4, path);
        uint32_t n = 0;
        readOrDie(f, &n, 4, path);
        std::fclose(f);
        return (int)n;
    }
    die("unsupported VAT version");
    return 0;
}

//==============================================================================
// Cage: voxel grid + Freudenthal 6-tet decomposition + face adjacency
//==============================================================================

// Corner bit layout: bit0 = +x, bit1 = +y, bit2 = +z.
// Same pattern in every voxel => all face diagonals are parallel across the
// grid => the tet mesh is conforming (Kuhn/Freudenthal triangulation).
static const int kTetsPerVoxel[6][4] = {
    {0, 1, 3, 7},  // 000, 100, 110, 111
    {0, 3, 2, 7},  // 000, 110, 010, 111
    {0, 2, 6, 7},  // 000, 010, 011, 111
    {0, 6, 4, 7},  // 000, 011, 001, 111
    {0, 4, 5, 7},  // 000, 001, 101, 111
    {0, 5, 1, 7},  // 000, 101, 100, 111
};

struct FaceKey {
    uint32_t v[3];
    bool operator==(const FaceKey& o) const {
        return v[0] == o.v[0] && v[1] == o.v[1] && v[2] == o.v[2];
    }
};
struct FaceKeyHash {
    size_t operator()(const FaceKey& k) const {
        size_t h = 1469598103934665603ull;
        for (uint32_t x : k.v) {
            h ^= (size_t)x;
            h *= 1099511628211ull;
        }
        return h;
    }
};

struct CageTet {
    uint32_t v[4];     // lattice vertex indices; face f is opposite v[f]
    int32_t nb[4];     // neighbor tet across face f, -1 = boundary
};

struct Cage {
    Vec3 origin;
    float h = 1.f;
    uint32_t nx = 0, ny = 0, nz = 0;
    std::vector<Vec3> verts;               // allocated lattice points
    std::vector<int32_t> lattice;          // (nx+1)(ny+1)(nz+1) -> vert idx | -1
    std::vector<int32_t> tetBase;          // per flat voxel: first tet id | -1 (inactive)
    std::vector<CageTet> tets;
    uint32_t activeVoxels = 0;

    uint32_t latticeId(uint32_t i, uint32_t j, uint32_t k) {
        size_t idx = ((size_t)k * (ny + 1) + j) * (nx + 1) + i;
        if (lattice[idx] < 0) {
            lattice[idx] = (int32_t)verts.size();
            verts.push_back({origin.x + (float)i * h, origin.y + (float)j * h,
                             origin.z + (float)k * h});
        }
        return (uint32_t)lattice[idx];
    }
};

static Cage buildCage(const Mesh& mesh, int res) {
    // Bounds
    Vec3 bmin{1e30f, 1e30f, 1e30f}, bmax{-1e30f, -1e30f, -1e30f};
    for (const auto& v : mesh.verts) {
        bmin.x = std::min(bmin.x, v.p.x); bmax.x = std::max(bmax.x, v.p.x);
        bmin.y = std::min(bmin.y, v.p.y); bmax.y = std::max(bmax.y, v.p.y);
        bmin.z = std::min(bmin.z, v.p.z); bmax.z = std::max(bmax.z, v.p.z);
    }
    float maxExt = std::max({bmax.x - bmin.x, bmax.y - bmin.y, bmax.z - bmin.z});
    if (maxExt <= 0.f) die("degenerate mesh bounds");
    float h = maxExt / (float)res;

    Cage cage;
    cage.h = h;
    cage.origin = bmin;
    cage.nx = std::max(1u, (uint32_t)std::ceil((bmax.x - bmin.x) / h));
    cage.ny = std::max(1u, (uint32_t)std::ceil((bmax.y - bmin.y) / h));
    cage.nz = std::max(1u, (uint32_t)std::ceil((bmax.z - bmin.z) / h));
    cage.lattice.assign(((size_t)cage.nx + 1) * (cage.ny + 1) * (cage.nz + 1), -1);

    // Active voxels = overlapped by any triangle AABB (padded by one cell)
    std::vector<uint8_t> active((size_t)cage.nx * cage.ny * cage.nz, 0);
    cage.tetBase.assign((size_t)cage.nx * cage.ny * cage.nz, -1);
    auto clampI = [](int v, int lo, int hi) { return std::max(lo, std::min(hi, v)); };
    for (const auto& t : mesh.tris) {
        const Vec3& pa = mesh.verts[t.a].p;
        const Vec3& pb = mesh.verts[t.b].p;
        const Vec3& pc = mesh.verts[t.c].p;
        Vec3 tmin{std::min({pa.x, pb.x, pc.x}), std::min({pa.y, pb.y, pc.y}),
                  std::min({pa.z, pb.z, pc.z})};
        Vec3 tmax{std::max({pa.x, pb.x, pc.x}), std::max({pa.y, pb.y, pc.y}),
                  std::max({pa.z, pb.z, pc.z})};
        int i0 = clampI((int)std::floor((tmin.x - bmin.x) / h) - 1, 0, (int)cage.nx - 1);
        int i1 = clampI((int)std::floor((tmax.x - bmin.x) / h) + 1, 0, (int)cage.nx - 1);
        int j0 = clampI((int)std::floor((tmin.y - bmin.y) / h) - 1, 0, (int)cage.ny - 1);
        int j1 = clampI((int)std::floor((tmax.y - bmin.y) / h) + 1, 0, (int)cage.ny - 1);
        int k0 = clampI((int)std::floor((tmin.z - bmin.z) / h) - 1, 0, (int)cage.nz - 1);
        int k1 = clampI((int)std::floor((tmax.z - bmin.z) / h) + 1, 0, (int)cage.nz - 1);
        for (int k = k0; k <= k1; ++k)
            for (int j = j0; j <= j1; ++j)
                for (int i = i0; i <= i1; ++i)
                    active[((size_t)k * cage.ny + j) * cage.nx + i] = 1;
    }

    // Create 6 tets per active voxel
    std::unordered_map<FaceKey, std::vector<std::pair<uint32_t, int>>, FaceKeyHash> faceMap;
    for (uint32_t k = 0; k < cage.nz; ++k)
        for (uint32_t j = 0; j < cage.ny; ++j)
            for (uint32_t i = 0; i < cage.nx; ++i) {
                if (!active[((size_t)k * cage.ny + j) * cage.nx + i]) continue;
                ++cage.activeVoxels;
                cage.tetBase[((size_t)k * cage.ny + j) * cage.nx + i] = (int32_t)cage.tets.size();
                uint32_t corner[8];
                for (int c = 0; c < 8; ++c)
                    corner[c] = cage.latticeId(i + ((c & 1) ? 1u : 0u),
                                               j + ((c & 2) ? 1u : 0u),
                                               k + ((c & 4) ? 1u : 0u));
                for (int t = 0; t < 6; ++t) {
                    CageTet ct{};
                    for (int v = 0; v < 4; ++v) ct.v[v] = corner[kTetsPerVoxel[t][v]];
                    ct.nb[0] = ct.nb[1] = ct.nb[2] = ct.nb[3] = -1;
                    uint32_t id = (uint32_t)cage.tets.size();
                    cage.tets.push_back(ct);
                    for (int f = 0; f < 4; ++f) {
                        FaceKey key{ct.v[(f + 1) & 3], ct.v[(f + 2) & 3], ct.v[(f + 3) & 3]};
                        std::sort(key.v, key.v + 3);
                        faceMap[key].push_back({id, f});
                    }
                }
            }

    // Resolve neighbors: shared faces appear exactly twice, boundary faces once
    for (const auto& [key, refs] : faceMap) {
        (void)key;
        if (refs.size() == 2) {
            cage.tets[refs[0].first].nb[refs[0].second] = (int32_t)refs[1].first;
            cage.tets[refs[1].first].nb[refs[1].second] = (int32_t)refs[0].first;
        } else if (refs.size() != 1) {
            die("internal error: tet face with " + std::to_string(refs.size()) + " owners");
        }
    }
    return cage;
}

//==============================================================================
// Quantized segments for cross-face pairing (defined early: clipping records
// dropped-sliver chords with them)
//==============================================================================

struct Seg {
    long long v[6];  // canonical: first endpoint <= second (lexicographic)
    // debug provenance (not part of the key)
    uint32_t frag = ~0u, edge = 0;
};
static Seg makeSeg(Vec3 a, Vec3 b, float q) {
    long long ax = (long long)std::llround(a.x / q), ay = (long long)std::llround(a.y / q),
              az = (long long)std::llround(a.z / q);
    long long bx = (long long)std::llround(b.x / q), by = (long long)std::llround(b.y / q),
              bz = (long long)std::llround(b.z / q);
    long long e0[3] = {ax, ay, az}, e1[3] = {bx, by, bz};
    if (std::lexicographical_compare(e1, e1 + 3, e0, e0 + 3)) std::swap(e0, e1);
    return Seg{e0[0], e0[1], e0[2], e1[0], e1[1], e1[2]};
}
static bool segLess(const Seg& a, const Seg& b) {
    for (int i = 0; i < 6; ++i) {
        if (a.v[i] != b.v[i]) return a.v[i] < b.v[i];
    }
    return false;
}
static bool segNear(const Seg& a, const Seg& b) {
    for (int i = 0; i < 6; ++i)
        if (std::llabs(a.v[i] - b.v[i]) > 1) return false;
    return true;
}
static void segCoords(char* buf, size_t n, const Seg& s, float q) {
    std::snprintf(buf, n, "(%.5f,%.5f,%.5f)-(%.5f,%.5f,%.5f)", s.v[0] * q, s.v[1] * q,
                  s.v[2] * q, s.v[3] * q, s.v[4] * q, s.v[5] * q);
}

//==============================================================================
// Clipping: triangle x 4 half-spaces -> convex polygon -> fan triangles
//==============================================================================

struct FragVert {
    Vec3 p, n;
    Vec2 uv;
    bool cut = false;       // inserted by a plane cut (vs. original source vertex)
    uint32_t pidSet[3] = {0, 0, 0};  // plane ids this cut vertex lies ON:
                                     // primary = inserting plane (exact by
                                     // construction), then coincident planes of
                                     // the same tet within epsExact. Measured
                                     // against the OWNING tet's plane
                                     // parameterization — never cross-tet, so
                                     // membership is symmetric across an
                                     // interface.
};

struct FragTri {
    uint32_t tet = 0;
    uint32_t srcTri = 0;
    int coplanarFace = -1;  // face slot the SOURCE triangle lies in (-1 = crossing)
    uint8_t chainMask = 0;  // bit e: edge (v[e], v[(e+1)%3]) is a polygon
                            // boundary edge (not a fan diagonal)
    FragVert v[3];
    bool dropped = false;   // removed by ownership resolution
};

struct BuildParams {
    float epsIn = 0.f;     // point kept if plane distance <= epsIn
    float epsCopl = 0.f;   // source-vertex counts as on-plane within this
    float epsOn = 0.f;     // pairing: ORIGINAL-vertex endpoint counts as on the
                           // face plane within this (cut vertices use pidSet
                           // membership instead — no cross-tet measurement)
    float epsExact = 0.f;  // pidSet secondary-plane coincidence tolerance
    float epsArea = 0.f;   // sliver triangle area threshold
    float qDist = 0.f;     // vertex dedup / pairing quantization bucket
};

// Global identity for cage face planes (sorted lattice triple). Shared between
// clipping and validation so cut-vertex provenance can be compared across the
// two tets that own a shared face.
struct PlaneIntern {
    std::unordered_map<FaceKey, uint32_t, FaceKeyHash> ids;
    uint32_t intern(const FaceKey& key) {
        auto it = ids.find(key);
        if (it != ids.end()) return it->second;
        uint32_t id = (uint32_t)ids.size() + 1u;  // 0 = "no plane"
        ids.emplace(key, id);
        return id;
    }
};

static FaceKey faceKeyOf(const Cage& cage, const CageTet& ct, int f) {
    FaceKey key{ct.v[(f + 1) & 3], ct.v[(f + 2) & 3], ct.v[(f + 3) & 3]};
    std::sort(key.v, key.v + 3);
    return key;
}

static float triArea(const FragVert& a, const FragVert& b, const FragVert& c) {
    return 0.5f * length(cross(b.p - a.p, c.p - a.p));
}

// Plane through 3 face verts; normal points AWAY from `opposite`.
struct Plane { Vec3 n; float d; };  // keep: dot(n, x) <= d + epsIn
static Plane makePlane(Vec3 a, Vec3 b, Vec3 c, Vec3 opposite) {
    Vec3 n = normalize(cross(b - a, c - a));
    if (dot(n, opposite - a) > 0.f) n = n * -1.f;
    return {n, dot(n, a)};
}

static std::vector<FragTri> clipIntoPieces(const Mesh& mesh, const Cage& cage,
                                           const BuildParams& bp, PlaneIntern& pi,
                                           std::vector<std::vector<Seg>>& sliverEdges,
                                           uint64_t& sliverDrops) {
    sliverEdges.assign(cage.tets.size(), {});
    std::vector<FragTri> frags;
    for (uint32_t ti = 0; ti < mesh.tris.size(); ++ti) {
        const Tri& tri = mesh.tris[ti];
        FragVert sv[3];
        for (int v = 0; v < 3; ++v) {
            sv[v].p = mesh.verts[(&tri.a)[v]].p;
            sv[v].n = mesh.verts[(&tri.a)[v]].n;
            sv[v].uv = mesh.verts[(&tri.a)[v]].uv;
            sv[v].cut = false;
        }
        // voxel range of the triangle
        Vec3 tmin{std::min({sv[0].p.x, sv[1].p.x, sv[2].p.x}),
                  std::min({sv[0].p.y, sv[1].p.y, sv[2].p.y}),
                  std::min({sv[0].p.z, sv[1].p.z, sv[2].p.z})};
        Vec3 tmax{std::max({sv[0].p.x, sv[1].p.x, sv[2].p.x}),
                  std::max({sv[0].p.y, sv[1].p.y, sv[2].p.y}),
                  std::max({sv[0].p.z, sv[1].p.z, sv[2].p.z})};
        int i0 = std::max(0, (int)std::floor((tmin.x - cage.origin.x) / cage.h));
        int i1 = std::min((int)cage.nx - 1, (int)std::floor((tmax.x - cage.origin.x) / cage.h));
        int j0 = std::max(0, (int)std::floor((tmin.y - cage.origin.y) / cage.h));
        int j1 = std::min((int)cage.ny - 1, (int)std::floor((tmax.y - cage.origin.y) / cage.h));
        int k0 = std::max(0, (int)std::floor((tmin.z - cage.origin.z) / cage.h));
        int k1 = std::min((int)cage.nz - 1, (int)std::floor((tmax.z - cage.origin.z) / cage.h));

        for (int k = k0; k <= k1; ++k)
            for (int j = j0; j <= j1; ++j)
                for (int i = i0; i <= i1; ++i) {
                    size_t voxel = ((size_t)k * cage.ny + j) * cage.nx + i;
                    int32_t base = cage.tetBase[voxel];  // -1 for inactive voxels
                    if (base < 0) continue;
                    for (int t = 0; t < 6; ++t) {
                        uint32_t tetId = (uint32_t)base + (uint32_t)t;
                        const CageTet& ct = cage.tets[tetId];
                        Vec3 vp[4] = {cage.verts[ct.v[0]], cage.verts[ct.v[1]],
                                      cage.verts[ct.v[2]], cage.verts[ct.v[3]]};
                        Plane tetPlanes[4];
                        uint32_t tetPlaneIds[4];
                        for (int f = 0; f < 4; ++f) {
                            tetPlanes[f] = makePlane(vp[(f + 1) & 3], vp[(f + 2) & 3],
                                                     vp[(f + 3) & 3], vp[f]);
                            tetPlaneIds[f] = pi.intern(faceKeyOf(cage, ct, f));
                        }

                        // coplanarity of the SOURCE triangle with one of the faces
                        int coplanar = -1;
                        for (int f = 0; f < 4 && coplanar < 0; ++f) {
                            const Plane& pl = tetPlanes[f];
                            bool allOn = true;
                            for (int v = 0; v < 3; ++v)
                                if (std::fabs(dot(pl.n, sv[v].p) - pl.d) > bp.epsCopl) {
                                    allOn = false;
                                    break;
                                }
                            if (allOn) coplanar = f;
                        }

                        // Sutherland–Hodgman against the 4 face planes.
                        // NOTE: the kept region must stay a half-space
                        // (d <= epsIn) — a band tie-break here would make it
                        // non-convex and fan triangulation could self-overlap.
                        std::vector<FragVert> poly(sv, sv + 3);
                        bool empty = false;
                        for (int f = 0; f < 4 && !empty; ++f) {
                            const Plane& pl = tetPlanes[f];
                            std::vector<FragVert> out;
                            out.reserve(poly.size() + 1);
                            for (size_t p = 0; p < poly.size(); ++p) {
                                const FragVert& a = poly[p];
                                const FragVert& b = poly[(p + 1) % poly.size()];
                                float da = dot(pl.n, a.p) - pl.d;
                                float db = dot(pl.n, b.p) - pl.d;
                                bool ain = da <= bp.epsIn;
                                bool bin = db <= bp.epsIn;
                                if (ain) out.push_back(a);
                                // Insert the plane cut ONLY on a strict sign
                                // straddle: t = da/(da-db) then lies in (0,1)
                                // by construction. Band-vs-out classification
                                // differences (both endpoints on the same side
                                // of the plane, one past epsIn) must NOT cut —
                                // interpolating at the plane there extrapolates
                                // a reflected vertex far off the edge.
                                bool cross = (da > 0.f && db < 0.f) ||
                                             (da < 0.f && db > 0.f);
                                if (ain != bin && cross) {
                                    float t = da / (da - db);
                                    FragVert c;
                                    c.p = a.p + (b.p - a.p) * t;
                                    c.n = normalize(a.n + (b.n - a.n) * t);
                                    c.uv = {a.uv.x + (b.uv.x - a.uv.x) * t,
                                            a.uv.y + (b.uv.y - a.uv.y) * t};
                                    c.cut = true;
                                    c.pidSet[0] = tetPlaneIds[f];  // primary: exact
                                    out.push_back(c);
                                }
                            }
                            poly.swap(out);
                            empty = poly.size() < 3;
                        }
                        if (empty) continue;

                        // pidSet secondaries: other tet planes the cut vertex
                        // coincides with (within float noise) — e.g. cut points
                        // landing on a lattice edge shared by two face planes.
                        for (auto& fv : poly) {
                            if (!fv.cut) continue;
                            int n = 1;
                            for (int f = 0; f < 4 && n < 3; ++f) {
                                if (tetPlaneIds[f] == fv.pidSet[0]) continue;
                                if (std::fabs(dot(tetPlanes[f].n, fv.p) - tetPlanes[f].d) <=
                                    bp.epsExact)
                                    fv.pidSet[n++] = tetPlaneIds[f];
                            }
                        }

                        // fan triangulation; mark polygon boundary ("chain")
                        // edges — fan diagonals are interior to the piece and
                        // must never participate in cross-face pairing
                        const size_t polyM = poly.size();
                        for (size_t p = 1; p + 1 < polyM; ++p) {
                            FragTri ft;
                            ft.tet = tetId;
                            ft.srcTri = ti;
                            ft.coplanarFace = coplanar;
                            ft.v[0] = poly[0];
                            ft.v[1] = poly[p];
                            ft.v[2] = poly[p + 1];
                            ft.chainMask = 2;                       // (v1, v2) always chain
                            if (p == 1) ft.chainMask |= 1;           // (v0, v1)
                            if (p + 2 == polyM) ft.chainMask |= 4;   // (v2, v0)
                            float area = triArea(ft.v[0], ft.v[1], ft.v[2]);
                            if (area < bp.epsArea) {
                                ++sliverDrops;
                                // keep the dropped sliver's boundary chords: an
                                // unmatched chord on the neighbor side whose
                                // counterpart fell below the area tolerance is
                                // tolerated by the pairing gate
                                for (int se = 0; se < 3; ++se)
                                    if (ft.chainMask & (1u << se))
                                        sliverEdges[tetId].push_back(makeSeg(
                                            ft.v[se].p, ft.v[(se + 1) % 3].p, bp.qDist));
                                continue;
                            }
                            frags.push_back(ft);
                        }
                    }
                }
    }
    return frags;
}

//==============================================================================
// Coplanar ownership: (srcTri, shared face) fragments kept by lowest tet id
//==============================================================================

struct OwnerKey {
    uint32_t tri, a, b, c;
    bool operator==(const OwnerKey& o) const {
        return tri == o.tri && a == o.a && b == o.b && c == o.c;
    }
};
struct OwnerKeyHash {
    size_t operator()(const OwnerKey& k) const {
        size_t h = 1469598103934665603ull;
        for (uint32_t x : {k.tri, k.a, k.b, k.c}) {
            h ^= (size_t)x;
            h *= 1099511628211ull;
        }
        return h;
    }
};

static size_t resolveOwnership(std::vector<FragTri>& frags, const Cage& cage) {
    std::unordered_map<OwnerKey, std::vector<uint32_t>, OwnerKeyHash> byOwner;
    for (uint32_t fi = 0; fi < frags.size(); ++fi) {
        const FragTri& ft = frags[fi];
        if (ft.coplanarFace < 0) continue;
        const CageTet& ct = cage.tets[ft.tet];
        int f = ft.coplanarFace;
        OwnerKey key{ft.srcTri, ct.v[(f + 1) & 3], ct.v[(f + 2) & 3], ct.v[(f + 3) & 3]};
        std::sort(&key.a, &key.a + 3);
        byOwner[key].push_back(fi);
    }
    size_t dropped = 0;
    for (auto& [key, list] : byOwner) {
        (void)key;
        if (list.size() <= 1) continue;
        uint32_t minTet = frags[list[0]].tet;
        for (uint32_t fi : list) minTet = std::min(minTet, frags[fi].tet);
        for (uint32_t fi : list)
            if (frags[fi].tet != minTet) {
                frags[fi].dropped = true;
                ++dropped;
            }
    }
    return dropped;
}

//==============================================================================
// Empty-tet drop + neighbor fixup
//==============================================================================

static std::vector<uint32_t> dropEmptyTets(std::vector<FragTri>& frags, Cage& cage,
                                           std::vector<std::vector<Seg>>& sliverEdges) {
    std::vector<uint32_t> fragCount(cage.tets.size(), 0);
    for (const auto& ft : frags)
        if (!ft.dropped) ++fragCount[ft.tet];

    std::vector<int32_t> remap(cage.tets.size(), -1);
    std::vector<CageTet> kept;
    kept.reserve(cage.tets.size());
    for (size_t t = 0; t < cage.tets.size(); ++t) {
        if (fragCount[t] == 0) continue;
        remap[t] = (int32_t)kept.size();
        kept.push_back(cage.tets[t]);
    }
    // fix neighbors: -1 for dropped partners, remap the rest
    for (auto& ct : kept)
        for (int f = 0; f < 4; ++f)
            if (ct.nb[f] >= 0) ct.nb[f] = remap[(size_t)ct.nb[f]];
    for (auto& ft : frags)
        if (!ft.dropped) ft.tet = (uint32_t)remap[ft.tet];
    {
        std::vector<std::vector<Seg>> mapped(kept.size());
        for (size_t t = 0; t < remap.size(); ++t)
            if (remap[t] >= 0) mapped[(size_t)remap[t]] = std::move(sliverEdges[t]);
        sliverEdges.swap(mapped);
    }
    cage.tets.swap(kept);

    std::vector<uint32_t> count(cage.tets.size(), 0);
    for (const auto& ft : frags)
        if (!ft.dropped) ++count[ft.tet];
    return count;
}

//==============================================================================
// Gate 1 validation
//==============================================================================

struct ValidationReport {
    bool pass = true;
    double sourceArea = 0.0, pieceArea = 0.0, areaRelErr = 0.0;
    uint64_t unmatchedSegs = 0, totalSegs = 0, toleratedSegs = 0;
    double sampleMaxDistToSrc = 0.0, sampleMaxDistToPieces = 0.0;
    uint64_t sampleFailures = 0;
    uint64_t debugExamples = 0;  // pairing mismatch examples printed so far
};

//---- closest point on triangle (Ericson, Real-Time Collision Detection)
static Vec3 closestPointOnTri(Vec3 p, Vec3 a, Vec3 b, Vec3 c) {
    Vec3 ab = b - a, ac = c - a, ap = p - a;
    float d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0.f && d2 <= 0.f) return a;
    Vec3 bp = p - b;
    float d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0.f && d4 <= d3) return b;
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f) {
        float v = d1 / (d1 - d3);
        return a + ab * v;
    }
    Vec3 cp = p - c;
    float d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0.f && d5 <= d6) return c;
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f) {
        float w = d2 / (d2 - d6);
        return a + ac * w;
    }
    float va = d3 * d6 - d5 * d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b) * w;
    }
    float denom = 1.f / (va + vb + vc);
    float v = vb * denom, w = vc * denom;
    return a + ab * v + ac * w;
}

//---- simple triangle hash grid for nearest-surface queries
struct TriGrid {
    float cell = 1.f;
    std::vector<Vec3> pa, pb, pc;
    struct CellKey {
        int x, y, z;
        bool operator==(const CellKey& o) const {
            return x == o.x && y == o.y && z == o.z;
        }
    };
    struct CellKeyHash {
        size_t operator()(const CellKey& k) const {
            size_t h = 1469598103934665603ull;
            for (int v : {k.x, k.y, k.z}) {
                h ^= (size_t)(int64_t)v;
                h *= 1099511628211ull;
            }
            return h;
        }
    };
    std::unordered_map<CellKey, std::vector<uint32_t>, CellKeyHash> cells;

    void addTri(Vec3 a, Vec3 b, Vec3 c) {
        uint32_t id = (uint32_t)pa.size();
        pa.push_back(a); pb.push_back(b); pc.push_back(c);
        Vec3 mn{std::min({a.x, b.x, c.x}), std::min({a.y, b.y, c.y}), std::min({a.z, b.z, c.z})};
        Vec3 mx{std::max({a.x, b.x, c.x}), std::max({a.y, b.y, c.y}), std::max({a.z, b.z, c.z})};
        for (int z = (int)std::floor(mn.z / cell); z <= (int)std::floor(mx.z / cell); ++z)
            for (int y = (int)std::floor(mn.y / cell); y <= (int)std::floor(mx.y / cell); ++y)
                for (int x = (int)std::floor(mn.x / cell); x <= (int)std::floor(mx.x / cell); ++x)
                    cells[{x, y, z}].push_back(id);
    }

    // nearest surface distance; searches rings until a hit and one padded ring
    // is covered. Returns -1 if nothing found within maxRing rings.
    double dist(Vec3 p, int maxRing = 24) const {
        int cx = (int)std::floor(p.x / cell), cy = (int)std::floor(p.y / cell),
            cz = (int)std::floor(p.z / cell);
        double best = 1e30;
        bool any = false;
        int ring = 0;
        while (ring <= maxRing) {
            for (int z = cz - ring; z <= cz + ring; ++z)
                for (int y = cy - ring; y <= cy + ring; ++y)
                    for (int x = cx - ring; x <= cx + ring; ++x) {
                        if (ring > 0 && std::abs(x - cx) != ring && std::abs(y - cy) != ring &&
                            std::abs(z - cz) != ring)
                            continue;  // ring shell only
                        auto it = cells.find({x, y, z});
                        if (it == cells.end()) continue;
                        for (uint32_t id : it->second) {
                            Vec3 q = closestPointOnTri(p, pa[id], pb[id], pc[id]);
                            double d = length(q - p);
                            if (d < best) best = d;
                            any = true;
                        }
                    }
            if (any && ring >= 1) break;
            if (any && best < (double)cell * 0.25) break;
            ++ring;
        }
        return any ? best : -1.0;
    }
};

//---- area-weighted uniform surface sampling
static std::vector<Vec3> sampleSurface(const std::vector<float>& triAreas,
                                       const std::vector<float>& areaCdf,
                                       const std::function<Vec3(uint32_t, float, float)>& pick,
                                       size_t count, uint64_t seed) {
    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<float> uni(0.f, 1.f);
    std::vector<Vec3> pts;
    pts.reserve(count);
    for (size_t s = 0; s < count; ++s) {
        float u = uni(rng);
        uint32_t ti = (uint32_t)(std::lower_bound(areaCdf.begin(), areaCdf.end(), u) -
                                 areaCdf.begin());
        ti = std::min(ti, (uint32_t)triAreas.size() - 1);
        float r1 = std::sqrt(uni(rng));
        float b0 = 1.f - r1, b1 = uni(rng) * r1;  // b2 = 1 - b0 - b1
        pts.push_back(pick(ti, b0, b1));
    }
    return pts;
}

struct SegHash {
    size_t operator()(const Seg& s) const {
        size_t h = 1469598103934665603ull;
        for (long long x : s.v) {
            h ^= (size_t)x;
            h *= 1099511628211ull;
        }
        return h;
    }
};
struct SegEq {
    bool operator()(const Seg& a, const Seg& b) const {
        return std::memcmp(a.v, b.v, sizeof a.v) == 0;
    }
};

static ValidationReport validate(const Mesh& mesh, const std::vector<FragTri>& frags,
                                 const Cage& cage, const BuildParams& bp, PlaneIntern& pi,
                                 const std::vector<std::vector<Seg>>& sliverEdges,
                                 size_t sampleCount, uint64_t seed) {
    ValidationReport r;

    //---- 1. area conservation
    double srcArea = 0.0, pieceArea = 0.0;
    for (const auto& t : mesh.tris)
        srcArea += 0.5 * (double)length(cross(mesh.verts[t.b].p - mesh.verts[t.a].p,
                                              mesh.verts[t.c].p - mesh.verts[t.a].p));
    for (const auto& ft : frags)
        if (!ft.dropped) pieceArea += triArea(ft.v[0], ft.v[1], ft.v[2]);
    r.sourceArea = srcArea;
    r.pieceArea = pieceArea;
    r.areaRelErr = std::fabs(pieceArea - srcArea) / std::max(srcArea, 1e-30);
    if (r.areaRelErr > 1e-4) r.pass = false;

    //---- 2. cross-face segment pairing
    // group frags by tet
    std::vector<std::vector<uint32_t>> byTet(cage.tets.size());
    for (uint32_t fi = 0; fi < frags.size(); ++fi)
        if (!frags[fi].dropped) byTet[frags[fi].tet].push_back(fi);

    struct FilterTotals {
        uint64_t straddleFrag = 0, straddleOk = 0, fanDiag = 0, onPlaneFail = 0,
                 degenerate = 0, kept = 0;
    } gFsTotals;
    for (uint32_t t = 0; t < cage.tets.size(); ++t)
        for (int f = 0; f < 4; ++f) {
            int32_t nb = cage.tets[t].nb[f];
            if (nb < 0 || (uint32_t)nb <= t) continue;  // each interior face once
            FilterTotals fs;
            const CageTet& ct = cage.tets[t];
            Plane pl = makePlane(cage.verts[ct.v[(f + 1) & 3]],
                                 cage.verts[ct.v[(f + 2) & 3]],
                                 cage.verts[ct.v[(f + 3) & 3]], cage.verts[ct.v[f]]);
            uint32_t P = pi.intern(faceKeyOf(cage, ct, f));
            auto onPlane = [&](Vec3 p) {
                return std::fabs(dot(pl.n, p) - pl.d) <= bp.epsOn;
            };
            // Endpoint chord-membership. Cut vertices use their pidSet (own-tet
            // plane identities — the inserting plane is exact by construction,
            // so no cross-tet epsilon is involved and the test is symmetric
            // across the interface). Original vertices are tested geometrically
            // with the SAME plane parameterization on both sides (pl comes from
            // tet t), so that test is also side-independent.
            auto endpointOn = [&](const FragVert& w) {
                if (w.cut) return w.pidSet[0] == P || w.pidSet[1] == P || w.pidSet[2] == P;
                return onPlane(w.p);
            };
            // An edge pairs at this face iff the source triangle STRADDLES the
            // face plane (surface genuinely crossing; tangency terminations and
            // coplanar-in-plane fragments have no partner by construction), the
            // edge is a boundary edge of the piece (count == 1; interior fan
            // edges appear twice), both endpoints lie in the face plane, and at
            // least one endpoint was PRODUCED by a cut on this plane.
            auto gather = [&](uint32_t tet, std::vector<Seg>& segs) {
                for (uint32_t fi : byTet[tet]) {
                    const FragTri& ft = frags[fi];
                    const Tri& st = mesh.tris[ft.srcTri];
                    const uint32_t* sv2 = &st.a;
                    bool neg = false, pos = false;
                    for (int v = 0; v < 3; ++v) {
                        float d = dot(pl.n, mesh.verts[sv2[v]].p) - pl.d;
                        if (d < -bp.epsCopl) neg = true;
                        if (d > +bp.epsCopl) pos = true;
                    }
                    if (!neg || !pos) {
                        fs.straddleFrag++;
                        continue;  // no crossing -> nothing to pair
                    }
                    fs.straddleOk++;
                    for (int e = 0; e < 3; ++e) {
                        if (!(ft.chainMask & (1u << e))) { fs.fanDiag++; continue; }
                        const FragVert& u = ft.v[e];
                        const FragVert& v = ft.v[(e + 1) % 3];
                        if (!endpointOn(u) || !endpointOn(v)) { fs.onPlaneFail++; continue; }
                        Seg s = makeSeg(u.p, v.p, bp.qDist);
                        s.frag = fi;
                        s.edge = (uint32_t)e;
                        if (s.v[0] == s.v[3] && s.v[1] == s.v[4] && s.v[2] == s.v[5]) {
                            fs.degenerate++;
                            continue;  // degenerate (below quantization)
                        }
                        segs.push_back(s);
                        ++r.totalSegs;
                        fs.kept++;
                    }
                }
            };
            std::vector<Seg> sa, sb;
            gather(t, sa);
            gather((uint32_t)nb, sb);
            gFsTotals.straddleFrag += fs.straddleFrag;
            gFsTotals.straddleOk += fs.straddleOk;
            gFsTotals.fanDiag += fs.fanDiag;
            gFsTotals.onPlaneFail += fs.onPlaneFail;
            gFsTotals.degenerate += fs.degenerate;
            gFsTotals.kept += fs.kept;
            if (sa.empty() && sb.empty()) continue;
            std::sort(sa.begin(), sa.end(), segLess);
            std::sort(sb.begin(), sb.end(), segLess);
            // Dedupe coincident chords within one side: a near-zero-thickness
            // wedge piece contributes the crossing chord twice (both long edges
            // within float noise of the plane), and coincident source sheets
            // (T-junctions / doubled panels) do the same. The surface crosses
            // the interface ONCE per chord — dedupe keeps the multiset pairing
            // meaningful. Both sides dedupe identically, so the check stays
            // symmetric.
            auto dedupNear = [](std::vector<Seg>& v) {
                if (v.empty()) return;
                std::vector<Seg> out;
                out.push_back(v[0]);
                for (size_t k = 1; k < v.size(); ++k)
                    if (!segNear(out.back(), v[k])) out.push_back(v[k]);
                v.swap(out);
            };
            dedupNear(sa);
            dedupNear(sb);
            // Windowed matching: a plain merge-with-tolerance two-pointer can
            // strand an element whose near-partner sits a few positions away;
            // scan forward for a partner instead.
            std::vector<Seg> unmatched;
            {
                std::vector<char> bUsed(sb.size(), 0);
                size_t j = 0;
                for (const Seg& a : sa) {
                    bool matched = false;
                    for (size_t k = j; k < sb.size(); ++k) {
                        if (bUsed[k]) continue;
                        if (segNear(a, sb[k])) {
                            bUsed[k] = 1;
                            matched = true;
                            while (j < sb.size() && bUsed[j]) ++j;
                            break;
                        }
                        if (segLess(a, sb[k])) break;  // sorted: past the window
                    }
                    if (!matched) unmatched.push_back(a);
                }
                for (size_t k = 0; k < sb.size(); ++k)
                    if (!bUsed[k]) unmatched.push_back(sb[k]);
            }
            size_t faceUnmatched = 0, faceTolerated = 0;
            for (const Seg& s : unmatched) {
                // A chord whose counterpart fell below the sliver area
                // tolerance on the other side is within the same tolerance the
                // cutter itself uses — accept it (recorded as tolerated).
                uint32_t other = (frags[s.frag].tet == t) ? (uint32_t)nb : t;
                bool tolerated = false;
                for (const Seg& d : sliverEdges[other])
                    if (segNear(s, d)) {
                        tolerated = true;
                        break;
                    }
                if (tolerated) ++faceTolerated;
                else ++faceUnmatched;
            }
            r.unmatchedSegs += faceUnmatched;
            r.toleratedSegs += faceTolerated;
            // print the first few offending faces with endpoint provenance
            if (faceUnmatched >= 4 && r.debugExamples < 8) {
                tcl("pair face t=%u f=%d nb=%d P=%u segs %zu/%zu unmatched %zu "
                            "tolerated %zu\n",
                            t, f, (int)nb, P, sa.size(), sb.size(), faceUnmatched,
                            faceTolerated);
                for (const Seg& s : unmatched) {
                    if (r.debugExamples >= 8) break;
                    const FragTri& ft = frags[s.frag];
                    const FragVert& u = ft.v[s.edge];
                    const FragVert& v = ft.v[(s.edge + 1) % 3];
                    char cb[192];
                    segCoords(cb, sizeof cb, s, bp.qDist);
                    tcl(
                        "  pair-mismatch %s tet=%u srcTri=%u copl=%d seg=%s "
                        "u(pids %u/%u/%u cut=%d) v(pids %u/%u/%u cut=%d)\n",
                        ft.tet == t ? "A" : "B", ft.tet, ft.srcTri, ft.coplanarFace, cb,
                        u.pidSet[0], u.pidSet[1], u.pidSet[2], (int)u.cut, v.pidSet[0],
                        v.pidSet[1], v.pidSet[2], (int)v.cut);
                    ++r.debugExamples;
                }
            }
        }
    // Pairing gate: area + sampling are the strict guardians; segment pairing
    // is defense-in-depth with a tolerance for the documented fp-boundary
    // residue (band vertices splitting a chord into collinear sub-edges on one
    // side; coincident source sheets quantizing differently across sides).
    // Both are one-sided SURPLUS, which cannot mask a crack — a real crack
    // loses partners in O(surface) quantity and fails area/sampling as well
    // (this is how the clip extrapolation bug was caught).
    if (r.unmatchedSegs > 8 && r.unmatchedSegs * 20ull > r.totalSegs)
        r.pass = false;
    tcl(
        "pair-stats: frags(straddle %llu, non-straddle %llu) edges(fanDiag %llu, "
        "onPlaneFail %llu, degenerate %llu, kept %llu)\n",
        (unsigned long long)gFsTotals.straddleOk, (unsigned long long)gFsTotals.straddleFrag,
        (unsigned long long)gFsTotals.fanDiag, (unsigned long long)gFsTotals.onPlaneFail,
        (unsigned long long)gFsTotals.degenerate, (unsigned long long)gFsTotals.kept);

    //---- 3. two-way surface sampling
    TriGrid srcGrid, pieceGrid;
    srcGrid.cell = cage.h;
    pieceGrid.cell = cage.h;
    for (const auto& t : mesh.tris)
        srcGrid.addTri(mesh.verts[t.a].p, mesh.verts[t.b].p, mesh.verts[t.c].p);
    for (const auto& ft : frags)
        if (!ft.dropped) pieceGrid.addTri(ft.v[0].p, ft.v[1].p, ft.v[2].p);

    std::vector<float> srcAreas(mesh.tris.size());
    std::vector<float> srcCdf;
    srcCdf.reserve(mesh.tris.size());
    double acc = 0.0;
    for (size_t t = 0; t < mesh.tris.size(); ++t) {
        double a = 0.5 * (double)length(cross(mesh.verts[mesh.tris[t].b].p - mesh.verts[mesh.tris[t].a].p,
                                              mesh.verts[mesh.tris[t].c].p - mesh.verts[mesh.tris[t].a].p));
        srcAreas[t] = (float)a;
        acc += a / std::max(srcArea, 1e-30);
        srcCdf.push_back((float)acc);
    }
    auto pickSrc = [&](uint32_t t, float b0, float b1) {
        float b2 = 1.f - b0 - b1;
        const Vert& a = mesh.verts[mesh.tris[t].a];
        const Vert& b = mesh.verts[mesh.tris[t].b];
        const Vert& c = mesh.verts[mesh.tris[t].c];
        return a.p * b0 + b.p * b1 + c.p * b2;
    };

    double tol = (double)cage.h * 1e-2;
    for (const auto& p : sampleSurface(srcAreas, srcCdf, pickSrc, sampleCount, seed)) {
        double d = pieceGrid.dist(p);
        if (d < 0) { ++r.sampleFailures; r.sampleMaxDistToPieces = 1e30; continue; }
        r.sampleMaxDistToPieces = std::max(r.sampleMaxDistToPieces, d);
        if (d > tol) ++r.sampleFailures;
    }
    size_t keptCount = 0;
    for (const auto& ft : frags)
        if (!ft.dropped) ++keptCount;
    // sample kept frags only; index through a compacted id list
    std::vector<uint32_t> keptIds;
    keptIds.reserve(keptCount);
    for (uint32_t fi = 0; fi < frags.size(); ++fi)
        if (!frags[fi].dropped) keptIds.push_back(fi);
    std::vector<float> keptAreas(keptIds.size());
    std::vector<float> keptCdf;
    keptCdf.reserve(keptIds.size());
    acc = 0.0;
    for (size_t i = 0; i < keptIds.size(); ++i) {
        const FragTri& ft = frags[keptIds[i]];
        keptAreas[i] = triArea(ft.v[0], ft.v[1], ft.v[2]);
        acc += keptAreas[i] / std::max(pieceArea, 1e-30);
        keptCdf.push_back((float)acc);
    }
    auto pickKept = [&](uint32_t i, float b0, float b1) {
        const FragTri& ft = frags[keptIds[i]];
        float b2 = 1.f - b0 - b1;
        return ft.v[0].p * b0 + ft.v[1].p * b1 + ft.v[2].p * b2;
    };
    for (const auto& p : sampleSurface(keptAreas, keptCdf, pickKept, sampleCount, seed ^ 0x9e3779b97f4a7c15ull)) {
        double d = srcGrid.dist(p);
        if (d < 0) { ++r.sampleFailures; r.sampleMaxDistToSrc = 1e30; continue; }
        r.sampleMaxDistToSrc = std::max(r.sampleMaxDistToSrc, d);
        if (d > tol) {
            ++r.sampleFailures;
            if (r.debugExamples < 6) {
                // locate the piece triangle this sample came from
                double best = 1e30;
                uint32_t bfi = ~0u;
                for (uint32_t fi : keptIds) {
                    const FragTri& ft = frags[fi];
                    Vec3 q = closestPointOnTri(p, ft.v[0].p, ft.v[1].p, ft.v[2].p);
                    double dd = length(q - p);
                    if (dd < best) {
                        best = dd;
                        bfi = fi;
                    }
                }
                const FragTri& ft = frags[bfi];
                const Tri& st2 = mesh.tris[ft.srcTri];
                const Vert& s0 = mesh.verts[st2.a];
                const Vert& s1 = mesh.verts[st2.b];
                const Vert& s2 = mesh.verts[st2.c];
                tcl(
                    "  spurious-sample d=%.4f p=(%.4f,%.4f,%.4f) tet=%u srcTri=%u "
                    "copl=%d\n    frag=(%.4f,%.4f,%.4f)/(%.4f,%.4f,%.4f)/(%.4f,%.4f,%.4f)\n"
                    "    src=(%.4f,%.4f,%.4f)/(%.4f,%.4f,%.4f)/(%.4f,%.4f,%.4f)\n",
                    d, p.x, p.y, p.z, ft.tet, ft.srcTri, ft.coplanarFace, ft.v[0].p.x,
                    ft.v[0].p.y, ft.v[0].p.z, ft.v[1].p.x, ft.v[1].p.y, ft.v[1].p.z,
                    ft.v[2].p.x, ft.v[2].p.y, ft.v[2].p.z, s0.p.x, s0.p.y, s0.p.z,
                    s1.p.x, s1.p.y, s1.p.z, s2.p.x, s2.p.y, s2.p.z);
                ++r.debugExamples;
            }
        }
    }
    if (r.sampleFailures != 0) r.pass = false;

    return r;
}

//==============================================================================
// Piece assembly (per-tet contiguous, local vertex dedup) + writers
//==============================================================================

struct PieceVert { Vec3 p, n; Vec2 uv; };

struct Assembly {
    std::vector<PieceVert> verts;      // contiguous per piece
    std::vector<uint32_t> tris;        // 3 local indices per piece triangle
    struct Range { uint32_t vertStart, vertCount, triStart, triCount; };
    std::vector<Range> ranges;         // per retained tet
};

static Assembly assemble(const std::vector<FragTri>& frags, const Cage& cage,
                         const BuildParams& bp) {
    Assembly as;
    as.ranges.resize(cage.tets.size());
    std::vector<std::vector<uint32_t>> byTet(cage.tets.size());
    for (uint32_t fi = 0; fi < frags.size(); ++fi)
        if (!frags[fi].dropped) byTet[frags[fi].tet].push_back(fi);

    for (uint32_t t = 0; t < cage.tets.size(); ++t) {
        auto& range = as.ranges[t];
        range.vertStart = (uint32_t)as.verts.size();
        range.triStart = (uint32_t)(as.tris.size() / 3);
        if (byTet[t].empty()) {
            range.vertCount = 0;
            range.triCount = 0;
            continue;
        }
        std::unordered_map<uint64_t, uint32_t> local;
        local.reserve(byTet[t].size() * 3);
        for (uint32_t fi : byTet[t]) {
            const FragTri& ft = frags[fi];
            uint32_t idx[3];
            for (int v = 0; v < 3; ++v) {
                // full-range quantized key (splitmix-style mix; no masking that
                // could alias distant coordinates at fine cage resolutions)
                uint64_t kx = (uint64_t)std::llround(ft.v[v].p.x / bp.qDist);
                uint64_t ky = (uint64_t)std::llround(ft.v[v].p.y / bp.qDist);
                uint64_t kz = (uint64_t)std::llround(ft.v[v].p.z / bp.qDist);
                uint64_t key = kx * 0x9E3779B97F4A7C15ull ^ ky * 0xC2B2AE3D27D4EB4Full ^
                               kz * 0x165667B19E3779F9ull;
                key ^= key >> 30; key *= 0xBF58476D1CE4E5B9ull;
                key ^= key >> 27; key *= 0x94D049BB133111EBull;
                key ^= key >> 31;
                auto it = local.find(key);
                if (it == local.end()) {
                    uint32_t id = range.vertStart + range.vertCount;
                    PieceVert pv{ft.v[v].p, ft.v[v].n, ft.v[v].uv};
                    as.verts.push_back(pv);
                    ++range.vertCount;
                    local.emplace(key, id);
                    idx[v] = id - range.vertStart;
                } else {
                    idx[v] = it->second - range.vertStart;
                }
            }
            as.tris.push_back(idx[0]);
            as.tris.push_back(idx[1]);
            as.tris.push_back(idx[2]);
            ++range.triCount;
        }
    }
    return as;
}

static void writeTetCage(const fs::path& path, const Cage& cage, const Assembly& as) {
    std::ofstream out(path, std::ios::binary);
    if (!out) die("cannot write " + path.string());
    auto wU32 = [&](uint32_t v) { out.write((const char*)&v, 4); };
    auto wI32 = [&](int32_t v) { out.write((const char*)&v, 4); };
    auto wF32 = [&](float v) { out.write((const char*)&v, 4); };

    out.write("TETCAGE1", 8);
    wU32(1);                                   // version
    wF32(cage.origin.x); wF32(cage.origin.y); wF32(cage.origin.z);
    wF32(cage.h);
    wU32(cage.nx); wU32(cage.ny); wU32(cage.nz);
    wU32((uint32_t)cage.verts.size());
    wU32((uint32_t)cage.tets.size());
    wU32((uint32_t)as.verts.size());
    wU32((uint32_t)(as.tris.size() / 3));
    wU32(0);                                   // frameCount (Stage 3)
    wU32(0);                                   // flags
    for (const auto& v : cage.verts) { wF32(v.x); wF32(v.y); wF32(v.z); }
    for (size_t ti = 0; ti < cage.tets.size(); ++ti) {
        const auto& t = cage.tets[ti];
        for (int i = 0; i < 4; ++i) wU32(t.v[i]);
        for (int i = 0; i < 4; ++i) wI32(t.nb[i]);
        const auto& r = as.ranges[ti];
        wU32(r.vertStart); wU32(r.vertCount); wU32(r.triStart); wU32(r.triCount);
    }
    for (const auto& v : as.verts) {
        wF32(v.p.x); wF32(v.p.y); wF32(v.p.z);
        wF32(v.n.x); wF32(v.n.y); wF32(v.n.z);
        wF32(v.uv.x); wF32(v.uv.y);
    }
    for (uint32_t i : as.tris) wU32(i);
    // NOTE: tet table is written in cage.tets order, and ranges[] was indexed by
    // the same order — the pointer arithmetic above recovers the index safely.
}

static void dumpPiecesOBJ(const fs::path& path, const Assembly& as) {
    std::ofstream out(path);
    if (!out) die("cannot write " + path.string());
    out << "# tet_cage piece dump\n";
    for (const auto& v : as.verts) {
        char buf[160];
        std::snprintf(buf, sizeof buf, "v %.6f %.6f %.6f\nvn %.6f %.6f %.6f\nvt %.6f %.6f\n",
                      v.p.x, v.p.y, v.p.z, v.n.x, v.n.y, v.n.z, v.uv.x, v.uv.y);
        out << buf;
    }
    for (size_t t = 0; t < as.ranges.size(); ++t) {
        const auto& r = as.ranges[t];
        if (r.triCount == 0) continue;
        out << "o tet_" << t << "\n";
        for (uint32_t i = 0; i < r.triCount; ++i) {
            uint32_t a = as.tris[(size_t)(r.triStart + i) * 3 + 0] + r.vertStart + 1;
            uint32_t b = as.tris[(size_t)(r.triStart + i) * 3 + 1] + r.vertStart + 1;
            uint32_t c = as.tris[(size_t)(r.triStart + i) * 3 + 2] + r.vertStart + 1;
            out << "f " << a << "/" << a << "/" << a << " " << b << "/" << b << "/" << b << " "
                << c << "/" << c << "/" << c << "\n";
        }
    }
}

//==============================================================================
// Driver: one topology end-to-end
//==============================================================================

struct RunResult {
    bool pass = false;
    ValidationReport vr;
    uint32_t tets = 0, cageVerts = 0, pieceVerts = 0, pieceTris = 0, activeVoxels = 0;
    uint64_t slivers = 0;
    size_t ownedDropped = 0;
    double seconds = 0.0;
};

static RunResult runTopology(const Mesh& mesh, int res, size_t samples, uint64_t seed,
                             const fs::path& outPath, const std::string& objOut) {
    RunResult rr;
    double t0 = nowSec();

    Cage cage = buildCage(mesh, res);

    BuildParams bp;
    bp.epsIn = cage.h * 1e-4f;
    bp.epsCopl = cage.h * 1e-4f;
    bp.epsOn = cage.h * 1e-5f;
    bp.epsExact = cage.h * 1e-5f;  // cross-side cut-point noise is ~1e-6; a
                                   // looser secondary test stays symmetric and
                                   // the per-side dedupe absorbs over-admission
    bp.epsArea = cage.h * cage.h * 1e-10f;
    bp.qDist = cage.h * 1e-3f;

    uint64_t slivers = 0;
    PlaneIntern pi;
    std::vector<std::vector<Seg>> sliverEdges;
    std::vector<FragTri> frags = clipIntoPieces(mesh, cage, bp, pi, sliverEdges, slivers);
    rr.slivers = slivers;

    rr.ownedDropped = resolveOwnership(frags, cage);
    auto fragCount = dropEmptyTets(frags, cage, sliverEdges);

    rr.vr = validate(mesh, frags, cage, bp, pi, sliverEdges, samples, seed);
    rr.pass = rr.vr.pass;

    Assembly as = assemble(frags, cage, bp);
    rr.tets = (uint32_t)cage.tets.size();
    rr.cageVerts = (uint32_t)cage.verts.size();
    rr.pieceVerts = (uint32_t)as.verts.size();
    rr.pieceTris = (uint32_t)(as.tris.size() / 3);
    rr.activeVoxels = cage.activeVoxels;

    writeTetCage(outPath, cage, as);
    if (!objOut.empty()) dumpPiecesOBJ(objOut, as);
    (void)fragCount;

    rr.seconds = nowSec() - t0;
    return rr;
}
