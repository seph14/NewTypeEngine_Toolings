//
// FBXImporter — standalone FBX -> NewTypeEngine asset converter.
//
// Windowed mode: ImGui panel with parameter toggles; CI_LOG_* output streams
// to the attached console and the in-app log panel.
// CLI mode (Bundler pattern): detect --input in setup(), run, exit.
//   FBXImporter.exe --input model.fbx [--out dir] [--hierarchy|--merge]
//                   [--optimize|--no-optimize] [--simplify 0.5]
//                   [--format trimesh|obj] [--no-textures] [--raw-units]
//                   [--dry-run]
//

#include "ConvertPipeline.h"

#include "cinder/CinderImGui.h"
#include "cinder/ImageIo.h"
#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/app/msw/PlatformMsw.h"
#include "cinder/gl/gl.h"
#include "cinder/log.h"

#include <deque>
#include <mutex>

using namespace ci;
using namespace ci::app;

namespace {

//------------------------------------------------------------------------------
// ImGui log panel — mirrors CI_LOG output into a ring buffer
//------------------------------------------------------------------------------

class PanelLogger : public log::Logger {
  public:
    void write(const log::Metadata& meta, const std::string& text) override {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mLines.size() >= 600) mLines.pop_front();
        mLines.push_back(text);
    }
    void draw() {
        std::lock_guard<std::mutex> lock(mMutex);
        ImGui::BeginChild("log", ImVec2(0, 0), ImGuiChildFlags_Borders,
            ImGuiWindowFlags_HorizontalScrollbar);
        for (const auto& line : mLines) {
            ImGui::TextUnformatted(line.c_str());
        }
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 4)
            ImGui::SetScrollHereY(1.0f);
        ImGui::EndChild();
    }
    void clear() {
        std::lock_guard<std::mutex> lock(mMutex);
        mLines.clear();
    }
  private:
    std::mutex mMutex;
    std::deque<std::string> mLines;
};

} // namespace

class FBXImporterApp : public App {
  public:
    void setup() override;
    void draw() override;
    void fileDrop(FileDropEvent event) override;

  private:
    int runCli(const std::vector<std::string>& args);
    void drawGui();
    void runConvert();

    fbximp::ConvertOptions mOptions;
    fbximp::ConvertResult mLastResult;
    bool mHasResult = false;
    std::shared_ptr<PanelLogger> mPanelLogger;
};

//------------------------------------------------------------------------------

void FBXImporterApp::setup() {
    // MSW routes CI_LOG console output to the debugger by default; direct it
    // to stdout so CLI runs (and the console attached to the GUI) show logs.
    static_cast<ci::app::PlatformMsw*>(ci::app::Platform::get())
        ->directConsoleToCout(true);

    const auto& args = getCommandLineArgs();

    bool cli = false;
    for (const auto& a : args)
        if (a == "--input") { cli = true; break; }

    // The log panel is useful in both modes (window stays open for the GUI).
    mPanelLogger = std::make_shared<PanelLogger>();
    log::manager()->addLogger(mPanelLogger);

    if (cli) {
        int code = runCli(args);
        // Console app: exit directly so build scripts see the real exit code.
        std::exit(code);
    }

    ImGui::Initialize();
    setWindowSize(720, 760);
}

int FBXImporterApp::runCli(const std::vector<std::string>& args) {
    fbximp::ConvertOptions opts;
    for (size_t i = 0; i < args.size(); i++) {
        const std::string& a = args[i];
        // MSW's getCommandLineArgs() includes argv[0]; skip anything that is
        // not an option (values are consumed via next()).
        if (a.rfind("--", 0) != 0) continue;
        auto next = [&](const std::string& name) -> std::string {
            if (i + 1 >= args.size()) {
                CI_LOG_E("Missing value for " << name);
                return {};
            }
            return args[++i];
        };
        if (a == "--input") opts.inputFbx = next(a);
        else if (a == "--out") opts.outDir = next(a);
        else if (a == "--hierarchy") opts.keepHierarchy = true;
        else if (a == "--merge") opts.keepHierarchy = false;
        else if (a == "--optimize") opts.optimize = true;
        else if (a == "--no-optimize") opts.optimize = false;
        else if (a == "--simplify") opts.simplifyRatio = std::stof(next(a));
        else if (a == "--format") opts.formatTriMesh = (next(a) == "trimesh");
        else if (a == "--no-textures") opts.extractTextures = false;
        else if (a == "--raw-units") opts.normalizeUnits = false;
        else if (a == "--dry-run") opts.dryRun = true;
        else {
            CI_LOG_E("Unknown argument: " << a);
            return 2;
        }
    }

    if (opts.inputFbx.empty()) {
        CI_LOG_E("--input <file.fbx> is required");
        return 2;
    }

    // Persistent per-run log next to the output (added alongside the console
    // logger so terminal echo is kept).
    std::error_code ec;
    fs::path outDir = opts.outDir.empty()
        ? opts.inputFbx.parent_path() / (opts.inputFbx.stem().string() + "_imported")
        : opts.outDir;
    fs::create_directories(outDir, ec);
    fs::path logPath = outDir / (opts.inputFbx.stem().string() + ".log");
    log::manager()->addLogger(std::make_shared<log::LoggerFile>(logPath, /*append=*/false));
    CI_LOG_I("FBXImporter log: " << logPath.string());

    fbximp::ConvertResult result = fbximp::convertFbx(opts);
    return result.ok ? 0 : 1;
}

void FBXImporterApp::fileDrop(FileDropEvent event) {
    if (!event.getFiles().empty()) {
        mOptions.inputFbx = event.getFiles()[0];
        CI_LOG_I("Input set to '" << mOptions.inputFbx.string() << "'");
    }
}

void FBXImporterApp::runConvert() {
    if (mOptions.inputFbx.empty()) {
        CI_LOG_E("No input FBX selected");
        return;
    }
    mLastResult = fbximp::convertFbx(mOptions);
    mHasResult = true;
}

void FBXImporterApp::draw() {
    gl::clear(Color(0.08f, 0.08f, 0.1f));
    drawGui();
}

void FBXImporterApp::drawGui() {
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize, ImGuiCond_Always);
    ImGui::Begin("FBX Importer", nullptr,
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

    if (ImGui::Button("Open FBX...")) {
        auto path = getOpenFilePath({}, { "fbx" });
        if (!path.empty()) mOptions.inputFbx = path;
    }
    ImGui::SameLine();
    if (ImGui::Button("Dry Run")) {
        mOptions.dryRun = true;
        runConvert();
        mOptions.dryRun = false;
    }
    ImGui::SameLine();
    if (ImGui::Button("Convert")) runConvert();
    ImGui::SameLine();
    if (ImGui::Button("Clear Log")) mPanelLogger->clear();

    if (!mOptions.inputFbx.empty())
        ImGui::Text("Input: %s", mOptions.inputFbx.string().c_str());
    else
        ImGui::Text("Input: (drop an .fbx here)");

    ImGui::Separator();
    ImGui::Text("Options");
    int mode = mOptions.keepHierarchy ? 1 : 0;
    if (ImGui::RadioButton("Merge by material (bake transforms)", &mode, 0))
        mOptions.keepHierarchy = false;
    ImGui::SameLine();
    if (ImGui::RadioButton("Keep hierarchy", &mode, 1))
        mOptions.keepHierarchy = true;

    ImGui::Checkbox("Optimize (meshoptimizer dedup/reorder)", &mOptions.optimize);
    if (mOptions.simplifyRatio > 0.f || ImGui::IsItemHovered())
        ImGui::SliderFloat("Simplify", &mOptions.simplifyRatio, 0.f, 1.f, "%.2f");
    else
        ImGui::SliderFloat("Simplify (0 = off)", &mOptions.simplifyRatio, 0.f, 1.f, "%.2f");

    int fmt = mOptions.formatTriMesh ? 0 : 1;
    if (ImGui::RadioButton("TriMesh binary (.msh)", &fmt, 0)) mOptions.formatTriMesh = true;
    ImGui::SameLine();
    if (ImGui::RadioButton("OBJ (.obj)", &fmt, 1)) mOptions.formatTriMesh = false;

    ImGui::Checkbox("Normalize to Y-up meters", &mOptions.normalizeUnits);
    ImGui::Checkbox("Extract textures (engine naming)", &mOptions.extractTextures);

    ImGui::Separator();
    if (mHasResult) {
        if (mLastResult.ok) {
            ImGui::Text("Last run OK: %d geometries, %d tris, %d verts, %d materials",
                (int)mLastResult.geometryCount, (int)mLastResult.triangleCount,
                (int)mLastResult.vertexCount, (int)mLastResult.materialCount);
            ImGui::Text("Output: %s", mLastResult.outDir.string().c_str());
        } else {
            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "Failed: %s", mLastResult.error.c_str());
        }
        ImGui::Separator();
    }

    ImGui::Text("Log");
    mPanelLogger->draw();
    ImGui::End();
}

CINDER_APP(FBXImporterApp, RendererGl,
    [](App::Settings* settings) {
        settings->setTitle("FBX Importer — NewTypeEngine");
        settings->setResizable(false);
    })
