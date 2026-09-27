//==============================================================================
// tet_cage - Stage-1 CLI driver (core in tet_cage_core.h, GUI in
// tet_cage_gui.cpp). Build/usage: see README.md in this directory.
//==============================================================================

#include "tet_cage_core.h"

int main(int argc, char** argv) {
    try {
    if (argc < 2) {
        std::fprintf(stderr,
                     "usage: tet_cage <input.(obj|vat)> [--topo i|-1] [--frame i] [--res r]\n"
                     "                    [--out path] [--objout path] [--samples n] [--seed s]\n"
                     "defaults: --topo 0 --frame 0 --res 16 --samples 200000\n");
        return 2;
    }
    fs::path input = argv[1];
    if (!fs::exists(input)) die("input not found: " + input.string());

    int topo = 0, frame = 0, res = 16;
    size_t samples = 200000;
    uint64_t seed = 0x5eed1234ull;
    fs::path out;
    std::string objOut;
    for (int i = 2; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) die(std::string(name) + " requires a value");
            return argv[++i];
        };
        if (a == "--topo") topo = std::atoi(next("--topo").c_str());
        else if (a == "--frame") frame = std::atoi(next("--frame").c_str());
        else if (a == "--res") res = std::atoi(next("--res").c_str());
        else if (a == "--samples") samples = (size_t)std::atoll(next("--samples").c_str());
        else if (a == "--seed") seed = std::strtoull(next("--seed").c_str(), nullptr, 10);
        else if (a == "--out") out = next("--out");
        else if (a == "--objout") objOut = next("--objout");
        else die("unknown argument " + a);
    }
    if (res < 1 || res > 256) die("--res must be in [1,256]");

    std::string ext = input.extension().string();
    for (auto& c : ext) c = (char)std::tolower((unsigned char)c);

    std::vector<int> topos;
    if (ext == ".vat") {
        int n = vatTopologyCount(input);
        if (topo == -1)
            for (int i = 0; i < n; ++i) topos.push_back(i);
        else
            topos.push_back(topo);
    } else {
        topos.push_back(0);
    }

    size_t totalPassed = 0;
    for (int t : topos) {
        Mesh mesh;
        if (ext == ".vat") {
            mesh = loadVAT(input, t, (uint32_t)frame);
        } else if (ext == ".obj") {
            if (t != topos.front()) break;
            mesh = loadOBJ(input);
        } else {
            die("unsupported input type " + ext + " (expected .obj or .vat)");
        }

        fs::path outPath = out;
        if (outPath.empty()) {
            outPath = input;
            if (ext == ".vat" && topos.size() > 1) {
                outPath.replace_filename(input.stem().string() + "_t" + std::to_string(t) +
                                         ".tetcage");
            } else {
                outPath.replace_extension(".tetcage");
            }
        }

        tcl("=== %s [topo %d, frame %d] ===\n", input.filename().string().c_str(), t, frame);
        tcl("source: %zu verts, %zu tris, area %.4f\n", mesh.verts.size(),
                    mesh.tris.size(), mesh.totalArea());

        RunResult rr = runTopology(mesh, res, samples, seed, outPath, objOut);
        tcl("cage:   res %d, %u active voxels, %u tets, %u cage verts "
                    "(%.1f tris/tet avg)\n",
                    res, rr.activeVoxels, rr.tets, rr.cageVerts,
                    rr.tets ? (double)rr.pieceTris / rr.tets : 0.0);
        tcl("pieces: %u verts, %u tris, slivers dropped %llu, ownership drops %zu\n",
                    rr.pieceVerts, rr.pieceTris, (unsigned long long)rr.slivers,
                    rr.ownedDropped);
        tcl("gate1:  area %.6f vs %.6f (rel err %.3e) | segs %llu unmatched %llu "
                    "tolerated %llu | samples maxD %.2e/%.2e fail %llu | %s | %.2fs\n",
                    rr.vr.pieceArea, rr.vr.sourceArea, rr.vr.areaRelErr,
                    (unsigned long long)rr.vr.totalSegs,
                    (unsigned long long)rr.vr.unmatchedSegs,
                    (unsigned long long)rr.vr.toleratedSegs,
                    rr.vr.sampleMaxDistToSrc, rr.vr.sampleMaxDistToPieces,
                    (unsigned long long)rr.vr.sampleFailures,
                    rr.pass ? "PASS" : "FAIL", rr.seconds);
        tcl("wrote:  %s\n", outPath.string().c_str());
        if (!objOut.empty()) tcl("dump:   %s\n", objOut.c_str());
        if (rr.pass) ++totalPassed;
    }

    tcl("\n%d/%d topologies passed Gate 1\n", (int)totalPassed, (int)topos.size());
    return totalPassed == topos.size() ? 0 : 1;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "%s\n", e.what());
        return 2;
    }
}
