//==============================================================================
// tet_cage_gui — Cinder/ImGui front-end for the Stage-1 cage builder.
// (app shell mirrors D:/Projects/FBXImporter)
//
// Windowed mode: left panel (input / parameters / build / stats / save / log),
// right viewport (source mesh, piece soup with per-tet coloring, cage
// wireframe, welded-boundary rims; drag to orbit, wheel to dolly).
//
// CLI passthrough (FBXImporter pattern — detects --input, runs headless):
//   tet_cage_gui.exe --input m.(obj|vat) [--topo i|-1] [--frame i] [--res r]
//                    [--out path] [--objout path] [--samples n] [--seed s]
//
// Core builder logic is shared with the CLI through tet_cage_core.h.
// =============================================================================

#include "tet_cage_core.h"

#include "cinder/CinderImGui.h"
#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/app/msw/PlatformMsw.h"
#include "cinder/gl/gl.h"

#include <deque>
#include <mutex>

using namespace ci;
using namespace ci::app;

namespace {

//------------------------------------------------------------------------------
// Log panel fed by the core's g_logSink (mirrors FBXImporter's PanelLogger)
//------------------------------------------------------------------------------
class PanelLogger {
  public:
    void add(const std::string& line) {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mLines.size() >= 600) mLines.pop_front();
        mLines.push_back(line);
    }
    void clear() {
        std::lock_guard<std::mutex> lock(mMutex);
        mLines.clear();
    }
    void draw() {
        std::lock_guard<std::mutex> lock(mMutex);
        ImGui::BeginChild("log", ImVec2(0, 0), ImGuiChildFlags_Borders,
                          ImGuiWindowFlags_HorizontalScrollbar);
        for (const auto& line : mLines) ImGui::TextUnformatted(line.c_str());
        if (!mLines.empty() && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 4)
            ImGui::SetScrollHereY(1.0f);
        ImGui::EndChild();
    }
  private:
    std::mutex mMutex;
    std::deque<std::string> mLines;
};

// One build's retained state (inputs + outputs + gate report).
struct GuiBuild {
    bool ok = false;
    Mesh src;
    Cage cage;
    Assembly as;
    uint64_t slivers = 0;
    size_t ownedDropped = 0;
    ValidationReport vr;
    double seconds = 0.0;
    std::vector<Vec3> cageLines;             // tet edges (pairs)
    std::vector<Vec3> rimLines;              // welded-boundary rims (pairs)
    size_t rimCount = 0;
};

// Welded-boundary rims of the piece soup (visualization port of
// tools/tet_cage/rim_partners.py; single-bucket quantized weld — includes the
// source mesh's own open boundary by design, see README).
std::vector<Vec3> guiWeldedRims(const Assembly& as, const Cage& cage) {
    const float tol = cage.h * 1e-5f;
    auto quant = [&](uint32_t v) -> uint64_t {
        const Vec3& p = as.verts[v].p;
        uint64_t kx = (uint64_t)std::llround(p.x / tol);
        uint64_t ky = (uint64_t)std::llround(p.y / tol);
        uint64_t kz = (uint64_t)std::llround(p.z / tol);
        uint64_t key = kx * 0x9E3779B97F4A7C15ull ^ ky * 0xC2B2AE3D27D4EB4Full ^
                       kz * 0x165667B19E3779F9ull;
        key ^= key >> 30; key *= 0xBF58476D1CE4E5B9ull;
        key ^= key >> 27; key *= 0x94D049BB133111EBull;
        key ^= key >> 31;
        return key;
    };
    std::unordered_map<uint64_t, uint32_t> weld;   // bucket -> representative id
    auto wid = [&](uint32_t v) -> uint32_t {
        uint64_t key = quant(v);
        auto it = weld.find(key);
        if (it != weld.end()) return it->second;
        weld.emplace(key, v);
        return v;
    };
    std::unordered_map<uint64_t, int> uses;
    for (size_t t = 0; t < cage.tets.size(); ++t) {
        const auto& r = as.ranges[t];
        for (uint32_t k = 0; k < r.triCount; ++k) {
            uint32_t a = as.tris[(size_t)(r.triStart + k) * 3 + 0] + r.vertStart;
            uint32_t b = as.tris[(size_t)(r.triStart + k) * 3 + 1] + r.vertStart;
            uint32_t c = as.tris[(size_t)(r.triStart + k) * 3 + 2] + r.vertStart;
            uint32_t w[3] = {wid(a), wid(b), wid(c)};
            for (int e = 0; e < 3; ++e) {
                uint32_t u = w[e], v = w[(e + 1) % 3];
                if (u == v) continue;
                uint64_t key = ((uint64_t)std::min(u, v) << 32) | std::max(u, v);
                ++uses[key];
            }
        }
    }
    std::vector<Vec3> rims;
    for (const auto& [key, n] : uses) {
        if (n != 1) continue;
        uint32_t u = (uint32_t)(key >> 32), v = (uint32_t)(key & 0xffffffffu);
        rims.push_back(as.verts[u].p);
        rims.push_back(as.verts[v].p);
    }
    return rims;
}

std::vector<Vec3> guiCageEdges(const Cage& cage) {
    std::vector<Vec3> lines;
    std::unordered_map<uint64_t, bool> seen;
    for (const auto& ct : cage.tets) {
        for (int e = 0; e < 4; ++e)
            for (int f = e + 1; f < 4; ++f) {
                uint32_t a = ct.v[e], b = ct.v[f];
                uint64_t key = ((uint64_t)std::min(a, b) << 32) | std::max(a, b);
                if (seen[key]) continue;
                seen[key] = true;
                lines.push_back(cage.verts[a]);
                lines.push_back(cage.verts[b]);
            }
    }
    return lines;
}

} // namespace

class TetCageApp : public App {
  public:
    void setup() override;
    void draw() override;
    void resize() override;
    void fileDrop(FileDropEvent event) override;
    void mouseDown(MouseEvent event) override;
    void mouseDrag(MouseEvent event) override;
    void mouseWheel(MouseEvent event) override;

  private:
    int runCli(const std::vector<std::string>& args);
    void loadInput(const fs::path& path);
    void build();
    void saveTetCage();
    void dumpObj();
    void rebuildBatches();
    void drawGui();
    void drawViewport();
    bool pointerInViewport(const vec2& pos) const;
    void updateCamera();

    // input + parameters
    fs::path mInput;
    int mTopo = 0, mFrame = 0, mRes = 16, mTopoCount = 1;
    PanelLogger mLog;

    // build results
    GuiBuild mB;
    bool mHasResult = false;
    std::string mError;

    // viewport
    static constexpr float kPanelW = 400.0f;
    CameraPersp mCam;
    vec3 mTarget = vec3(0);
    float mCamDist = 1.0f, mYaw = 0.6f, mPitch = 0.45f;
    vec2 mLastMouse;
    Rectf mViewRect{0, 0, 0, 0};
    gl::BatchRef mSrcBatch, mPieceBatch;
    bool mShowSrc = true, mShowPieces = true, mShowCage = false, mShowRims = false;
    bool mPerTetColor = true, mWireframe = false;
};

// --------------------------------------------------------------- setup/CLI ---

void TetCageApp::setup() {
    static_cast<ci::app::PlatformMsw*>(ci::app::Platform::get())
        ->directConsoleToCout(true);

    const auto& args = getCommandLineArgs();
    bool cli = false;
    for (const auto& a : args)
        if (a == "--input") { cli = true; break; }

    g_logSink = [&](const std::string& s) { mLog.add(s); };

    if (cli) {
        int code = runCli(args);
        std::exit(code);
    }

    ImGui::Initialize();
    setWindowSize(1280, 800);
    updateCamera();
}

int TetCageApp::runCli(const std::vector<std::string>& args) {
    fs::path input, out, objout;
    int topo = 0, frame = 0, res = 16;
    size_t samples = 200000;
    uint64_t seed = 0x5eed1234ull;
    for (size_t i = 0; i < args.size(); ++i) {
        const std::string& a = args[i];
        if (a.rfind("--", 0) != 0) continue;
        auto next = [&](const std::string& name) -> std::string {
            if (i + 1 >= args.size()) {
                tcl("tet_cage_gui: error: missing value for %s\n", name.c_str());
                std::exit(2);
            }
            return args[++i];
        };
        if (a == "--input") input = next(a);
        else if (a == "--topo") topo = std::atoi(next(a).c_str());
        else if (a == "--frame") frame = std::atoi(next(a).c_str());
        else if (a == "--res") res = std::atoi(next(a).c_str());
        else if (a == "--out") out = next(a);
        else if (a == "--objout") objout = next(a);
        else if (a == "--samples") samples = (size_t)std::atoll(next(a).c_str());
        else if (a == "--seed") seed = std::strtoull(next(a).c_str(), nullptr, 10);
        else {
            tcl("tet_cage_gui: error: unknown argument %s\n", a.c_str());
            return 2;
        }
    }
    if (input.empty()) {
        tcl("tet_cage_gui: error: --input <file.(obj|vat)> is required\n");
        return 2;
    }
    try {
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
        size_t passed = 0;
        for (int t : topos) {
            Mesh mesh = (ext == ".vat") ? loadVAT(input, t, (uint32_t)frame)
                                        : loadOBJ(input);
            fs::path outPath = out;
            if (outPath.empty()) {
                outPath = input;
                if (ext == ".vat" && topos.size() > 1)
                    outPath.replace_filename(
                        input.stem().string() + "_t" + std::to_string(t) + ".tetcage");
                else
                    outPath.replace_extension(".tetcage");
            }
            tcl("=== %s [topo %d, frame %d] ===\n",
                input.filename().string().c_str(), t, frame);
            RunResult rr = runTopology(mesh, res, samples, seed, outPath,
                                       objout.string());
            if (rr.pass) ++passed;
        }
        tcl("\n%d/%d topologies passed Gate 1\n", (int)passed, (int)topos.size());
        return passed == topos.size() ? 0 : 1;
    } catch (const std::exception& e) {
        tcl("%s\n", e.what());
        return 2;
    }
}

// ------------------------------------------------------------------ build ---

void TetCageApp::loadInput(const fs::path& path) {
    std::string ext = path.extension().string();
    for (auto& c : ext) c = (char)std::tolower((unsigned char)c);
    if (ext != ".vat" && ext != ".obj") {
        mLog.add("tet_cage_gui: unsupported input type '" + ext + "' (want .obj/.vat)\n");
        return;
    }
    mInput = path;
    try {
        mTopoCount = (ext == ".vat") ? vatTopologyCount(path) : 1;
    } catch (const std::exception& e) {
        mLog.add(std::string(e.what()) + "\n");
        mTopoCount = 1;
    }
    if (mTopo >= mTopoCount) mTopo = 0;
    mHasResult = false;
    mLog.add("Input set to '" + path.string() + "' (" +
             std::to_string(mTopoCount) + (mTopoCount == 1 ? " topology)\n"
                                                           : " topologies)\n"));
}

void TetCageApp::build() {
    if (mInput.empty()) {
        mLog.add("tet_cage_gui: no input selected\n");
        return;
    }
    mError.clear();
    try {
        double t0 = nowSec();
        std::string ext = mInput.extension().string();
        for (auto& c : ext) c = (char)std::tolower((unsigned char)c);

        GuiBuild b;
        b.src = (ext == ".vat") ? loadVAT(mInput, mTopo, (uint32_t)mFrame)
                                : loadOBJ(mInput);
        tcl("=== %s [topo %d, frame %d] ===\n",
            mInput.filename().string().c_str(), mTopo, mFrame);
        tcl("source: %zu verts, %zu tris, area %.4f\n", b.src.verts.size(),
            b.src.tris.size(), b.src.totalArea());

        b.cage = buildCage(b.src, mRes);

        BuildParams bp;
        bp.epsIn = b.cage.h * 1e-4f;
        bp.epsCopl = b.cage.h * 1e-4f;
        bp.epsOn = b.cage.h * 1e-5f;
        bp.epsExact = b.cage.h * 1e-5f;
        bp.epsArea = b.cage.h * b.cage.h * 1e-10f;
        bp.qDist = b.cage.h * 1e-3f;

        PlaneIntern pi;
        std::vector<std::vector<Seg>> sliverEdges;
        auto frags = clipIntoPieces(b.src, b.cage, bp, pi, sliverEdges, b.slivers);
        b.ownedDropped = resolveOwnership(frags, b.cage);
        auto fragCount = dropEmptyTets(frags, b.cage, sliverEdges);
        (void)fragCount;
        b.vr = validate(b.src, frags, b.cage, bp, pi, sliverEdges, 200000,
                        0x5eed1234ull);
        b.as = assemble(frags, b.cage, bp);
        b.seconds = nowSec() - t0;
        b.cageLines = guiCageEdges(b.cage);
        b.rimLines = guiWeldedRims(b.as, b.cage);
        b.rimCount = b.rimLines.size() / 2;
        b.ok = true;
        mB = std::move(b);
        mHasResult = true;

        // fit camera to the new content
        Vec3 bmin{1e30f, 1e30f, 1e30f}, bmax{-1e30f, -1e30f, -1e30f};
        for (const auto& v : mB.src.verts) {
            bmin.x = std::min(bmin.x, v.p.x); bmax.x = std::max(bmax.x, v.p.x);
            bmin.y = std::min(bmin.y, v.p.y); bmax.y = std::max(bmax.y, v.p.y);
            bmin.z = std::min(bmin.z, v.p.z); bmax.z = std::max(bmax.z, v.p.z);
        }
        mTarget = vec3((bmin.x + bmax.x) * 0.5f, (bmin.y + bmax.y) * 0.5f,
                       (bmin.z + bmax.z) * 0.5f);
        mCamDist = 2.5f * length(vec3(bmax.x - bmin.x, bmax.y - bmin.y,
                                      bmax.z - bmin.z));
        mCamDist = std::max(mCamDist, 1e-3f);
        updateCamera();
        rebuildBatches();
    } catch (const std::exception& e) {
        mError = e.what();
        mLog.add(mError + "\n");
        mHasResult = false;
    }
}

void TetCageApp::rebuildBatches() {
    if (!mHasResult) {
        mSrcBatch.reset();
        mPieceBatch.reset();
        return;
    }
    auto offset = [&](const Vec3& p) {
        return vec3(p.x, p.y, p.z) - mTarget;
    };
    // source mesh (uniform warm gray)
    {
        TriMesh::Format fmt = TriMesh::Format().positions().normals().colors();
        TriMesh tm(fmt);
        std::vector<vec3> pos, nrm;
        std::vector<Color> col;
        pos.reserve(mB.src.tris.size() * 3);
        nrm.reserve(mB.src.tris.size() * 3);
        col.reserve(mB.src.tris.size() * 3);
        for (const auto& t : mB.src.tris) {
            const Vert* corner[3] = {&mB.src.verts[t.a], &mB.src.verts[t.b],
                                     &mB.src.verts[t.c]};
            for (int c = 0; c < 3; ++c) {
                pos.push_back(offset(corner[c]->p));
                nrm.push_back(vec3(corner[c]->n.x, corner[c]->n.y, corner[c]->n.z));
                col.push_back(Color(0.62f, 0.58f, 0.52f));
            }
        }
        tm.appendPositions(pos.data(), pos.size());
        tm.appendNormals(nrm.data(), nrm.size());
        tm.appendColors(col.data(), col.size());
        mSrcBatch = gl::Batch::create(
            gl::VboMesh::create(tm), gl::getStockShader(gl::ShaderDef().color().lambert()));
    }
    // piece soup (flat per-tet hues make the fragmentation visible)
    {
        TriMesh::Format fmt = TriMesh::Format().positions().normals().colors();
        TriMesh tm(fmt);
        std::vector<vec3> pos, nrm;
        std::vector<Color> col;
        for (size_t t = 0; t < mB.cage.tets.size(); ++t) {
            const auto& r = mB.as.ranges[t];
            float hue = std::fmod(0.61803398875f * (float)t, 1.0f);
            Color c = mPerTetColor ? Color(CM_HSV, vec3(hue, 0.55f, 0.95f))
                                   : Color(0.55f, 0.75f, 0.6f);
            for (uint32_t k = 0; k < r.triCount; ++k) {
                uint32_t idx[3] = {
                    mB.as.tris[(size_t)(r.triStart + k) * 3 + 0] + r.vertStart,
                    mB.as.tris[(size_t)(r.triStart + k) * 3 + 1] + r.vertStart,
                    mB.as.tris[(size_t)(r.triStart + k) * 3 + 2] + r.vertStart};
                for (int v = 0; v < 3; ++v) {
                    const PieceVert& pv = mB.as.verts[idx[v]];
                    pos.push_back(offset(pv.p));
                    nrm.push_back(vec3(pv.n.x, pv.n.y, pv.n.z));
                    col.push_back(c);
                }
            }
        }
        tm.appendPositions(pos.data(), pos.size());
        tm.appendNormals(nrm.data(), nrm.size());
        tm.appendColors(col.data(), col.size());
        mPieceBatch = gl::Batch::create(
            gl::VboMesh::create(tm), gl::getStockShader(gl::ShaderDef().color().lambert()));
    }
}

void TetCageApp::saveTetCage() {
    if (!mHasResult) return;
    fs::path def = mInput;
    def.replace_extension(".tetcage");
    auto path = getSaveFilePath(def, {"tetcage"});
    if (path.empty()) return;
    try {
        writeTetCage(path, mB.cage, mB.as);
        mLog.add("wrote: " + path.string() + "\n");
    } catch (const std::exception& e) {
        mLog.add(std::string(e.what()) + "\n");
    }
}

void TetCageApp::dumpObj() {
    if (!mHasResult) return;
    auto path = getSaveFilePath(mInput.parent_path() / "pieces.obj", {"obj"});
    if (path.empty()) return;
    try {
        dumpPiecesOBJ(path, mB.as);
        mLog.add("dump: " + path.string() + "\n");
    } catch (const std::exception& e) {
        mLog.add(std::string(e.what()) + "\n");
    }
}

// ------------------------------------------------------------------- view ---

bool TetCageApp::pointerInViewport(const vec2& pos) const {
    return pos.x >= kPanelW && mViewRect.calcArea() > 0;
}

void TetCageApp::updateCamera() {
    vec3 dir(std::cos(mPitch) * std::sin(mYaw), std::sin(mPitch),
             std::cos(mPitch) * std::cos(mYaw));
    mCam.setEyePoint(mTarget + dir * mCamDist);
    mCam.lookAt(mTarget);
    float aspect = mViewRect.calcArea() > 0
                       ? mViewRect.getWidth() / std::max(mViewRect.getHeight(), 1.0f)
                       : 1.0f;
    mCam.setPerspective(45.0f, aspect, mCamDist * 0.001f, mCamDist * 100.0f);
}

void TetCageApp::resize() {
    updateCamera();
}

void TetCageApp::mouseDown(MouseEvent event) {
    mLastMouse = event.getPos();
}

void TetCageApp::mouseDrag(MouseEvent event) {
    if (ImGui::GetIO().WantCaptureMouse || !pointerInViewport(event.getPos())) {
        mLastMouse = event.getPos();
        return;
    }
    vec2 d = event.getPos() - mLastMouse;
    mLastMouse = event.getPos();
    mYaw -= 0.007f * d.x;
    mPitch = std::clamp(mPitch + 0.007f * d.y, -1.5f, 1.5f);
    updateCamera();
}

void TetCageApp::mouseWheel(MouseEvent event) {
    if (ImGui::GetIO().WantCaptureMouse || !pointerInViewport(event.getPos()))
        return;
    float w = event.getWheelIncrement();
    if (w == 0.0f) return;
    mCamDist *= std::pow(1.15f, -w);
    mCamDist = std::clamp(mCamDist, 1e-4f, 1e6f);
    updateCamera();
}

void TetCageApp::fileDrop(FileDropEvent event) {
    if (!event.getFiles().empty()) loadInput(event.getFiles()[0]);
}

void TetCageApp::draw() {
    gl::clear(Color(0.08f, 0.08f, 0.1f));
    drawGui();
    drawViewport();
}

void TetCageApp::drawViewport() {
    const ivec2 win = getWindowSize();
    Rectf view(kPanelW, 0.0f, (float)win.x, (float)win.y);
    if (view.getWidth() <= 0 || view.getHeight() <= 0) return;
    mViewRect = view;

    gl::ScopedViewport scp(ivec2((int)view.getX1(),
                                 (int)((float)win.y - view.getY2())),
                           ivec2((int)view.getWidth(), (int)view.getHeight()));
    gl::setMatrices(mCam);
    gl::ScopedDepthTest sdt(true);

    // small world-origin axis triad for orientation (x red, y green, z blue)
    {
        float a = 0.15f * mCamDist;
        gl::VertBatch axes(GL_LINES);
        axes.color(0.7f, 0.2f, 0.2f);
        axes.vertex(vec3(-mTarget) - vec3(a, 0, 0));
        axes.vertex(vec3(-mTarget) + vec3(a, 0, 0));
        axes.color(0.2f, 0.7f, 0.2f);
        axes.vertex(vec3(-mTarget) - vec3(0, a, 0));
        axes.vertex(vec3(-mTarget) + vec3(0, a, 0));
        axes.color(0.2f, 0.3f, 0.8f);
        axes.vertex(vec3(-mTarget) - vec3(0, 0, a));
        axes.vertex(vec3(-mTarget) + vec3(0, 0, a));
        axes.draw();
    }

    if (mHasResult) {
        auto offset = [&](const Vec3& p) { return vec3(p.x, p.y, p.z) - mTarget; };
        gl::ScopedModelMatrix smm;
        if (mWireframe) gl::enableWireframe();
        if (mShowPieces && mPieceBatch) mPieceBatch->draw();
        if (mShowSrc && mSrcBatch) mSrcBatch->draw();
        if (mWireframe) gl::disableWireframe();

        if (mShowCage && !mB.cageLines.empty()) {
            gl::color(0.2f, 0.9f, 0.4f);
            gl::VertBatch batch(GL_LINES);
            for (const Vec3& p : mB.cageLines) batch.vertex(offset(p));
            batch.draw();
        }
        if (mShowRims && !mB.rimLines.empty()) {
            gl::color(0.95f, 0.25f, 0.2f);
            gl::VertBatch batch(GL_LINES);
            for (const Vec3& p : mB.rimLines) batch.vertex(offset(p));
            batch.draw();
        }
    }
}

// -------------------------------------------------------------------- gui ---

void TetCageApp::drawGui() {
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(kPanelW, (float)getWindowHeight()),
                             ImGuiCond_Always);
    ImGui::Begin("tet_cage", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                                          ImGuiWindowFlags_NoCollapse);

    if (ImGui::Button("Open...###open")) {
        auto path = getOpenFilePath({}, {"vat", "obj"});
        if (!path.empty()) loadInput(path);
    }
    ImGui::SameLine();
    if (ImGui::Button("Build")) build();
    ImGui::SameLine();
    if (ImGui::Button("Clear Log")) mLog.clear();

    if (!mInput.empty())
        ImGui::TextWrapped("Input: %s", mInput.string().c_str());
    else
        ImGui::TextWrapped("Input: (drop an .obj/.vat here)");

    ImGui::SeparatorText("Parameters");
    ImGui::Text("topologies in file: %d", mTopoCount);
    ImGui::SliderInt("topology", &mTopo, 0, std::max(mTopoCount - 1, 0));
    ImGui::SliderInt("frame", &mFrame, 0, 4096);
    ImGui::SliderInt("res (voxels/longest axis)", &mRes, 2, 64);
    ImGui::TextWrapped("tets/piece tris grow ~quadratically with res "
                       "(paper uses 6-25 per axis)");

    ImGui::SeparatorText("View");
    ImGui::Checkbox("source mesh", &mShowSrc);
    ImGui::Checkbox("piece soup", &mShowPieces);
    ImGui::SameLine();
    ImGui::Checkbox("per-tet colors", &mPerTetColor);
    ImGui::Checkbox("cage wireframe", &mShowCage);
    ImGui::SameLine();
    ImGui::Checkbox("wireframe", &mWireframe);
    ImGui::Checkbox("boundary rims (red)", &mShowRims);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("includes the source's own open boundary; "
                          "see README / design doc Follow-up 3");

    if (mHasResult) {
        ImGui::SeparatorText("Result");
        ImGui::Text("source: %d verts, %d tris", (int)mB.src.verts.size(),
                    (int)mB.src.tris.size());
        ImGui::Text("cage:   h=%.4g, %d tets, %d cage verts", mB.cage.h,
                    (int)mB.cage.tets.size(), (int)mB.cage.verts.size());
        ImGui::Text("pieces: %d verts, %d tris", (int)mB.as.verts.size(),
                    (int)(mB.as.tris.size() / 3));
        ImGui::Text("slivers dropped: %llu, ownership drops: %zu",
                    (unsigned long long)mB.slivers, mB.ownedDropped);
        ImGui::Text("welded boundary rims: %zu", mB.rimCount);
        ImGui::Text("area rel err: %.3e | unmatched segs: %llu | sample fail: %llu",
                    mB.vr.areaRelErr, (unsigned long long)mB.vr.unmatchedSegs,
                    (unsigned long long)mB.vr.sampleFailures);
        if (mB.vr.pass)
            ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1), "Gate 1: PASS (%.2fs)",
                               mB.seconds);
        else
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1), "Gate 1: FAIL (%.2fs)",
                               mB.seconds);
        if (ImGui::Button("Save .tetcage...")) saveTetCage();
        ImGui::SameLine();
        if (ImGui::Button("Dump piece OBJ...")) dumpObj();
    } else if (!mError.empty()) {
        ImGui::SeparatorText("Result");
        ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "%s", mError.c_str());
    }

    ImGui::SeparatorText("Log");
    mLog.draw();
    ImGui::End();
}

CINDER_APP(TetCageApp, RendererGl,
           [](App::Settings* settings) {
               settings->setTitle("tet_cage - cage builder");
               settings->setResizable(true);
           })
