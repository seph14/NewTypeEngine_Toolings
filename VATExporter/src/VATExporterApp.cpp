#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/TriMesh.h"
#include "cinder/Utilities.h"
#include "cinder/ObjLoader.h"
#include "cinder/CinderImGui.h"
#include "cinder/FileWatcher.h"
#include "cinder/Camera.h"
#include "cinder/CameraUi.h"
#include "cinder/Log.h"
#include "xatlas.h"
#include <cctype>
#include "Resources.h"

using namespace ci;
using namespace ci::app;
using namespace std;

static const int TexcoordIdx = 1;
static const int NormalIdx = 2;

class VATExporterApp : public App {
public:
	vector<gl::VboRef>              mMeshIndex, mMeshUv, mMeshNormal;
    vector<gl::VaoRef>				mMeshVao;
    vector<gl::BufferTextureRef>	mVertBuffer, mNormalBuffer;
    vector<uvec4>                   mData;

	CameraPersp     mCam;
	CameraUi	    mCamUi;
    gl::GlslProgRef mRenderShader;

    bool		    mModelLoaded, mExport;
    bool            mHeadless;     // CLI mode: parse + write, skip the viewer upload
    bool            mUseV1;        // export format: V1 packed single file (true) or V0 per-topology files (false)
    float		    mSpeed,	mDirection, mTime, mPrevTime;
    uint32_t	    mVertCnt, mIndexCnt, mFrameCnt, mPrevVert, mPrevIdxCnt, mBundleIdx, mInstanceCnt;
    string		    mModelName;
    fs::path        mInputPath, mOutputPath;
    AxisAlignedBox mBoundBox;

	// V0: one file per topology, <mModelName><bundleIdx>.vat inside the output folder
	void writeVAT0(const uint32_t& bundleIdx,
                   const vector<GLuint>& indices,
                   const vector<vec3>& verts,
                   const vector<vec3>& normals,
                   const vector<vec2>& uv, uint32_t vertCnt,
                   const vec3& center, const vec3& extend);
    // V1: all topologies packed into one file behind a header table
    void writeVAT1(const fs::path& filePath,
                   const vector<vector<vec3>>& combinedVerts,
                   const vector<vector<vec3>>& combinedNormals,
                   const vector<vector<vec2>>& combinedTexcoords,
                   const vector<vector<GLuint>>& combinedIndices,
                   const vector<vec3>& centers, const vector<vec3>& extends);

    fs::path        vat1OutputPath() const;   // mOutputPath as explicit .vat file (or dir/<name>.vat)
    fs::path        vat0OutputDir() const;    // mOutputPath as folder receiving the numbered files
    uint32_t        countInputFrames(const fs::path& folderPath);   // contiguous 0.obj, 1.obj, ...
    bool            runFromCommandLine();     // true if a CLI run (or usage error) happened - caller quits
	void loadModel(const fs::path folderPath);

	void setup() override;
	void mouseDown( MouseEvent event ) override;
	void update() override;
	void draw() override;
};

void VATExporterApp::loadModel(const fs::path folderPath) {
    // reset any previous load
    mData.clear();
    mVertBuffer.clear();    mNormalBuffer.clear();
    mMeshIndex.clear();     mMeshVao.clear();
    mMeshUv.clear();        mMeshNormal.clear();
    mBundleIdx = 0;

    vector<vector<vec3>>    combinedVerts, combinedNormals;
    vector<vector<vec2>>    combinedTexcoords;
    vector<vector<GLuint>>  combinedIndices;
    vector<vec3>            combinedCenters, combinedExtends;
    vec3 extend, center;

    if (mExport) {
        mPrevVert   = 0;
        mPrevIdxCnt = 0;
        uint32_t modelIdx = 0, totalFrm = 0, currFrm = 0;
        vec3 bundleCenter, bundleExtend;   // bounds of the bundle currently accumulating

        vector<vec3> verts;
        vector<vec3> normals;
        vector<vec2> texcoords;
        vector<GLuint> indices;

        for (int i = 0; i < mFrameCnt; i++) {
            ObjLoader loader(loadFile(folderPath.string() + "/" + toString(i) + ".obj"));
            TriMeshRef mesh = TriMesh::create(loader);// >> geom::Scale::Scale(vec3(10.0f)));
            mVertCnt        = mesh->getNumVertices();

            // check if we have a new bundle
            if (mVertCnt != mPrevVert || mesh->getNumIndices() != mPrevIdxCnt) {
                auto boundbox = mesh->calcBoundingBox();
                extend        = boundbox.getExtents();
                center        = boundbox.getCenter();
                if (mPrevVert == 0)
                    mBoundBox = boundbox;
                else mBoundBox.include(boundbox);

                auto data = mesh->getIndices();
                mIndexCnt = data.size();

                // close the current bundle (files are written after the whole sequence is parsed)
                if (mPrevVert != 0) {
                    CI_LOG_I("Close model bundle: " << modelIdx << " [" << mPrevIdxCnt << "::" << mPrevVert << " / " << currFrm << "]");

                    mData.push_back            (uvec4(mPrevIdxCnt, mPrevVert, currFrm, totalFrm));
                    combinedVerts.push_back     (verts);
                    combinedNormals.push_back   (normals);
                    combinedTexcoords.push_back (texcoords);
                    combinedIndices.push_back   (indices);
                    combinedCenters.push_back   (bundleCenter);
                    combinedExtends.push_back   (bundleExtend);
                    verts.clear();
                    normals.clear();
                    texcoords.clear();
                    indices.clear();
                    modelIdx++;
                }

                currFrm = 0;
                bundleCenter = center;      // the new bundle starts with this frame's bounds
                bundleExtend = extend;
                indices.insert(indices.end(), data.begin(), data.end());
                CI_LOG_I("Process new model bundle: " << modelIdx << " [" << mIndexCnt << "::" << mVertCnt << "]");

                if (!mesh->hasTexCoords()) {
                    xatlas::Atlas* atlas = xatlas::Create();

                    xatlas::MeshDecl meshDecl;
                    meshDecl.vertexCount            = mVertCnt;
                    meshDecl.vertexPositionData     = mesh->getBufferPositions().data();
                    meshDecl.vertexPositionStride   = sizeof(float) * 3;
                    meshDecl.vertexNormalData       = mesh->getNormals().data();
                    meshDecl.vertexNormalStride     = sizeof(float) * 3;

                    meshDecl.indexCount     = (uint32_t)mesh->getNumIndices();
                    meshDecl.indexData      = mesh->getIndices().data();
                    meshDecl.indexFormat    = xatlas::IndexFormat::UInt32;
                    xatlas::AddMeshError::Enum error = xatlas::AddMesh(atlas, meshDecl, 1);

                    if (error != xatlas::AddMeshError::Success) {
                        xatlas::Destroy(atlas);
                        CI_LOG_I("Xatlas failed to generate UV: " << error);
                    } else {
                        xatlas::Generate(atlas);
                        CI_LOG_I("UV charts: " << atlas->chartCount << ", atlases: " << atlas->atlasCount);
                        for (uint32_t i = 0; i < atlas->atlasCount; i++)
                            CI_LOG_I(i << ":" << (atlas->utilization[i] * 100.0f) << " utilization");
                        CI_LOG_I("UV resolution: " << atlas->width << "," << atlas->height);
                        
                        texcoords.resize(mVertCnt);
                        if (atlas->width > 0 && atlas->height > 0) {
                            for (uint32_t i = 0; i < atlas->meshCount; i++) {
                                const xatlas::Mesh& xmesh = atlas->meshes[i];
                                // Rasterize mesh triangles.
                                for (uint32_t j = 0; j < xmesh.indexCount; j += 3) {
                                    int32_t atlasIndex = -1;
                                    int verts[3][2];
                                    for (int k = 0; k < 3; k++) {
                                        uint32_t vertIdx = xmesh.indexArray[j + k];
                                        uint32_t uvIdx = mesh->getIndices()[j + k];

                                        const xatlas::Vertex& v = xmesh.vertexArray[vertIdx];
                                        atlasIndex = v.atlasIndex; // The same for every vertex in the triangle.
                                        verts[k][0] = int(v.uv[0]);
                                        verts[k][1] = int(v.uv[1]);

                                        texcoords[uvIdx] = vec2(v.uv[0] / atlas->width, 1.f - v.uv[1] / atlas->height);
                                    }

                                    if (atlasIndex < 0)
                                        continue; // Skip triangles that weren't atlased.
                                }
                            }
                        }

                        // Cleanup.
                        xatlas::Destroy(atlas);
                    }
                } else {
                    auto uv0 = mesh->getTexCoords0<2>();
                    for (int v = 0; v < mVertCnt; v++) {
                        auto uv = uv0[v];
                        texcoords.push_back(uv);
                    }
                }

                mPrevVert   = mVertCnt;
                mPrevIdxCnt = mIndexCnt;

                // keep the UV block dense even if the atlas failed to generate
                if (texcoords.size() < mVertCnt)
                    texcoords.resize(mVertCnt, vec2(0.f));
            }

            // verts
            auto buffer = mesh->getBufferPositions();
            for (int v = 0; v < mVertCnt; v++)
                verts.push_back(vec3(buffer[3 * v + 0], buffer[3 * v + 1], buffer[3 * v + 2]));

            // Check if normals are valid (non-zero)
            bool hasValidNormals = mesh->hasNormals();
            if (hasValidNormals) {
                auto norm = mesh->getNormals();
                bool allZero = true;
                for (int v = 0; v < mVertCnt && allZero; v++) {
                    if (length(norm[v]) > 0.001f) {
                        allZero = false;
                        break;
                    }
                }
                if (allZero) {
                    hasValidNormals = false;
                    CI_LOG_W("Frame " << i << ": All normals are zero, recalculating...");
                }
            }

            if (!hasValidNormals)
                mesh->recalculateNormals(true, true);

            // normals
            auto norm = mesh->getNormals();
            for (int v = 0; v < mVertCnt; v++) {
                auto normal = norm[v];
                normals.push_back(normal);
            }

            totalFrm++;
            currFrm++;
        }

        if (verts.size() > 0) {
            CI_LOG_I("Close model bundle: " << modelIdx << " [" << mPrevIdxCnt << "::" << mPrevVert << " / " << currFrm << "]");
            mData.push_back(uvec4(mPrevIdxCnt, mPrevVert, currFrm, totalFrm));
            combinedVerts.push_back     (verts);
            combinedNormals.push_back   (normals);
            combinedTexcoords.push_back (texcoords);
            combinedIndices.push_back   (indices);
            combinedCenters.push_back   (bundleCenter);
            combinedExtends.push_back   (bundleExtend);
        }

        // write the parsed bundles to disk
        if (!mData.empty()) {
            if (mUseV1) {
                writeVAT1(vat1OutputPath(), combinedVerts, combinedNormals, combinedTexcoords,
                          combinedIndices, combinedCenters, combinedExtends);
            } else {
                for (uint32_t bundle = 0; bundle < combinedIndices.size(); bundle++)
                    writeVAT0(bundle, combinedIndices[bundle], combinedVerts[bundle], combinedNormals[bundle],
                              combinedTexcoords[bundle], mData[bundle].y, combinedCenters[bundle], combinedExtends[bundle]);
            }
        }
    } else {
        // a single packed V1 file takes precedence over the numbered V0 files
        fs::path packedPath = fs::path(mInputPath) / (mModelName + ".vat");
        fs::path firstPath  = fs::path(mInputPath) / (mModelName + "0.vat");
        if (!fs::is_regular_file(packedPath) && !fs::is_regular_file(firstPath)) {
            CI_LOG_E("No vat files found in " << mInputPath.string() << " for model name " << mModelName);
            return;
        }

        uint32_t totalFrm = 0;
        if (fs::is_regular_file(packedPath)) {
            auto target       = loadFile(packedPath.string());
            IStreamRef stream = target->createStream();
            uint32_t version = 0, topologyCnt = 0;
            stream->readLittle(&version);
            CI_ASSERT(version == 1);
            stream->readLittle(&topologyCnt);

            // header table: bounds + counts per topology
            vector<vec3>     centers(topologyCnt), extends(topologyCnt);
            vector<uint32_t> indexCnts(topologyCnt), vertCnts(topologyCnt), frameCnts(topologyCnt);
            for (uint32_t t = 0; t < topologyCnt; t++) {
                stream->readData(&centers[t].x, 3 * sizeof(float));
                stream->readData(&extends[t].x, 3 * sizeof(float));
                stream->readLittle(&indexCnts[t]);
                stream->readLittle(&vertCnts[t]);
                stream->readLittle(&frameCnts[t]);
            }

            for (uint32_t t = 0; t < topologyCnt; t++) {
                vector<GLuint> indices(indexCnts[t]);
                stream->readData(indices.data(), indexCnts[t] * sizeof(uint32_t));

                vector<vec2> texcoords(vertCnts[t]);
                stream->readData(texcoords.data(), 2 * vertCnts[t] * sizeof(float));

                uint32_t totalFloats = 3 * vertCnts[t] * frameCnts[t];
                vector<vec3> verts(totalFloats / 3), normals(totalFloats / 3);
                stream->readData(verts.data(),   totalFloats * sizeof(float));
                stream->readData(normals.data(), totalFloats * sizeof(float));

                AxisAlignedBox box(centers[t] - extends[t], centers[t] + extends[t]);
                if (t == 0) mBoundBox = box;
                else mBoundBox.include(box);

                mData.push_back(uvec4(indexCnts[t], vertCnts[t], frameCnts[t], totalFrm));
                combinedIndices.push_back  (move(indices));
                combinedVerts.push_back    (move(verts));
                combinedNormals.push_back  (move(normals));
                combinedTexcoords.push_back(move(texcoords));
                totalFrm += frameCnts[t];

                CI_LOG_I("Model bundle " << t << " - [" << vertCnts[t] << "::" << frameCnts[t] << "::" << totalFrm << "]");
            }
            CI_ASSERT(stream->tell() == stream->size());
        } else {
            // legacy V0: one file per topology, counted contiguously
            uint32_t fileCnt = 0;
            while (fs::is_regular_file(fs::path(mInputPath) / (mModelName + toString(fileCnt) + ".vat")))
                fileCnt++;

            for (uint32_t frm = 0; frm < fileCnt; frm++) {
                auto target       = loadFile((fs::path(mInputPath) / (mModelName + toString(frm) + ".vat")).string());
                IStreamRef stream = target->createStream();
                uint32_t version = 0;
                stream->readLittle(&version);
                CI_ASSERT(version == 0);

                vector<GLuint> indices;
                vector<vec3> verts, normals;
                vector<vec2> texcoords;

                //size
                stream->readData(&center.x, 3 * sizeof(float));
                stream->readData(&extend.x, 3 * sizeof(float));

                if (frm == 0)
                    mBoundBox = AxisAlignedBox(center - extend, center + extend);
                else mBoundBox.include(AxisAlignedBox(center - extend, center + extend));

                //indices
                stream->readLittle(&mIndexCnt);
                indices.resize(mIndexCnt);
                stream->readData(indices.data(), mIndexCnt * sizeof(uint32_t));

                //vert cnt
                stream->readLittle(&mVertCnt);
                texcoords.resize  (mVertCnt);
                stream->readData  (texcoords.data(), 2 * mVertCnt * sizeof(float));

                //vertices
                uint32_t totalVert = 0;
                stream->readLittle(&totalVert);
                verts.resize      (totalVert / 3);
                normals.resize    (totalVert / 3);

                stream->readData(verts.data(), totalVert * sizeof(float));
                stream->readData(normals.data(), totalVert * sizeof(float));

                mData.push_back(uvec4(mIndexCnt, mVertCnt, (totalVert / 3 / mVertCnt), totalFrm));
                combinedIndices.push_back  (indices);
                combinedVerts.push_back    (verts);
                combinedNormals.push_back  (normals);
                combinedTexcoords.push_back(texcoords);
                totalFrm += totalVert / mVertCnt / 3;

                CI_LOG_I("Model bundle " << frm << " - [" << mVertCnt << "::" << (totalVert / mVertCnt / 3) << "::" << totalFrm << "]");
            }
        }
    }

    // CLI mode: the viewer resources are not needed
    if (mHeadless)
        return;

    for (uint32_t bundle = 0; bundle < combinedIndices.size(); bundle++) {
        auto& verts     = combinedVerts[bundle];
        auto& normals   = combinedNormals[bundle];
        auto& texcoords = combinedTexcoords[bundle];
        auto& indices   = combinedIndices[bundle];

        // bake verts and normals into a texture buffer
        auto posVbo     = gl::Vbo::create(GL_ARRAY_BUFFER, verts.size() * sizeof(vec3), verts.data(), GL_STATIC_DRAW);
        auto vertBuffer = gl::BufferTexture::create(posVbo, GL_RGB32F);
        auto normal     = gl::Vbo::create(GL_ARRAY_BUFFER, normals.size() * sizeof(vec3), normals.data(), GL_STATIC_DRAW);
        auto normalBuffer = gl::BufferTexture::create(normal, GL_RGB32F);

        // save indices into vao
        auto indiceVbo  = gl::Vbo::create(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(), GL_STATIC_DRAW);
        auto uv         = gl::Vbo::create(GL_ARRAY_BUFFER, texcoords.size() * sizeof(vec2), texcoords.data(), GL_STATIC_DRAW);
        auto meshVbo    = ci::gl::Vao::create();

        {
            meshVbo->bind();
            uv->bind();
            gl::vertexAttribPointer(TexcoordIdx, 2, GL_FLOAT, GL_FALSE, 0, NULL);
            gl::enableVertexAttribArray(TexcoordIdx);
            meshVbo->unbind();
        }

        mVertBuffer.push_back   (vertBuffer);
        mNormalBuffer.push_back (normalBuffer);
        mMeshIndex.push_back    (indiceVbo);
        mMeshVao.push_back      (meshVbo);
    }

    mModelLoaded = true;
}

fs::path VATExporterApp::vat1OutputPath() const {
    if (mOutputPath.extension() == ".vat")
        return mOutputPath;
    return mOutputPath / (mModelName + ".vat");
}

fs::path VATExporterApp::vat0OutputDir() const {
    if (mOutputPath.extension() == ".vat")
        return mOutputPath.parent_path();
    return mOutputPath;
}

uint32_t VATExporterApp::countInputFrames(const fs::path& folderPath) {
    // frames are numbered contiguously: 0.obj, 1.obj, ... - stray files are ignored
    mFrameCnt = 0;
    while (fs::is_regular_file(folderPath / (toString(mFrameCnt) + ".obj")))
        mFrameCnt++;
    return mFrameCnt;
}

void VATExporterApp::writeVAT0(const uint32_t& bundleIdx,
                               const vector<GLuint>& indices, const vector<vec3>& verts,
                               const vector<vec3>& normals, const vector<vec2>& uv,
                               uint32_t vertCnt, const vec3& center, const vec3& extend) {
    if (!mExport) return;

    auto filePath = vat0OutputDir() / (mModelName + toString(bundleIdx) + ".vat");
    try {
        auto target     = writeFile(filePath.string());
        OStreamRef out  = target->getStream();

        // version
        out->writeLittle<uint32_t>(0);

        //bound box
        out->writeData(&center.x, 3 * sizeof(float));
        out->writeData(&extend.x, 3 * sizeof(float));

        //indices
        out->writeLittle(static_cast<uint32_t>(indices.size()));
        out->writeData  (indices.data(), indices.size() * sizeof(uint32_t));

        //vert count
        out->writeLittle(vertCnt);

        //uv
        out->writeData(uv.data(), 2 * vertCnt * sizeof(float));

        //positions
        out->writeLittle((uint32_t)(3 * verts.size()));
        out->writeData  (verts.data(), 3 * verts.size() * sizeof(float));    // pos
        out->writeData  (normals.data(), 3 * normals.size() * sizeof(float));// normals
        CI_LOG_I("Model bundle " << bundleIdx << " saved (V0): " << filePath.filename().string());
    } catch (const std::exception& ex) {
        CI_LOG_E("Failed to write " << filePath.string() << " - " << ex.what());
    }
}

void VATExporterApp::writeVAT1(const fs::path& filePath,
                               const vector<vector<vec3>>& combinedVerts,
                               const vector<vector<vec3>>& combinedNormals,
                               const vector<vector<vec2>>& combinedTexcoords,
                               const vector<vector<GLuint>>& combinedIndices,
                               const vector<vec3>& centers, const vector<vec3>& extends) {
    if (!mExport) return;

    const uint32_t count = (uint32_t)combinedIndices.size();
    try {
        auto target     = writeFile(filePath.string());
        OStreamRef out  = target->getStream();

        // version + topology count
        out->writeLittle<uint32_t>(1);
        out->writeLittle<uint32_t>(count);

        // header table: bounds + counts per topology
        for (uint32_t bundle = 0; bundle < count; bundle++) {
            out->writeData(&centers[bundle].x, 3 * sizeof(float));
            out->writeData(&extends[bundle].x, 3 * sizeof(float));
            out->writeLittle(static_cast<uint32_t>(combinedIndices[bundle].size()));
            out->writeLittle(mData[bundle].y);   // vertex count
            out->writeLittle(mData[bundle].z);   // frame count
        }

        // payloads in table order
        for (uint32_t bundle = 0; bundle < count; bundle++) {
            out->writeData(combinedIndices[bundle].data(),  combinedIndices[bundle].size() * sizeof(uint32_t));
            out->writeData(combinedTexcoords[bundle].data(), combinedTexcoords[bundle].size() * sizeof(vec2));
            out->writeData(combinedVerts[bundle].data(),    combinedVerts[bundle].size() * sizeof(vec3));
            out->writeData(combinedNormals[bundle].data(),  combinedNormals[bundle].size() * sizeof(vec3));
        }
        CI_LOG_I("VAT V1 saved: " << filePath.filename().string() << " [" << count << " topologies]");
    } catch (const std::exception& ex) {
        CI_LOG_E("Failed to write " << filePath.string() << " - " << ex.what());
    }
}

bool VATExporterApp::runFromCommandLine() {
    // CLI mode: VATExporter.exe --input <dir|frame.obj> [--out ...] [--name ...] [--v0]
    // Build-script friendly; without --input the app falls through to the GUI.
    const auto& args = getCommandLineArgs();
    fs::path input, out;
    string name;
    bool haveInput = false, useV0 = false;

    auto printUsage = [this]() {
        console() << "VATExporter - Vertex Animation Texture exporter" << endl
                  << "Input: a folder of numbered OBJ frames (0.obj, 1.obj, ...)" << endl << endl
                  << "Usage: VATExporter.exe --input <dir|frame.obj> [--out <dir|file.vat>] [--name <base>] [--v0] [--help]" << endl
                  << "  --input  folder containing the OBJ frame sequence, or one frame file (its folder is used)" << endl
                  << "  --out    output directory or exact .vat file path (V1); default: the input folder" << endl
                  << "  --name   base name of the output; default: the input folder name" << endl
                  << "  --v0     legacy format: one .vat file per topology (<name><N>.vat); default is V1 packed (<name>.vat)" << endl
                  << "  --help   show this help" << endl;
    };

    for (size_t i = 1; i < args.size(); ++i) {
        const string& arg = args[i];
        if      (arg == "--input" && i + 1 < args.size()) input = args[++i], haveInput = true;
        else if (arg == "--out"   && i + 1 < args.size()) out   = args[++i];
        else if (arg == "--name"  && i + 1 < args.size()) name  = args[++i];
        else if (arg == "--v0")                             useV0 = true;
        else if (arg == "--help" || arg == "-h")          { printUsage(); return true; }
        else {
            console() << "Unknown argument: " << arg << endl << endl;
            printUsage();
            return true;
        }
    }

    if (!haveInput)
        return false;   // GUI mode

    if (fs::is_regular_file(input))
        mInputPath = input.parent_path();
    else if (fs::is_directory(input))
        mInputPath = input;
    else {
        console() << "[ERROR] --input path not found: " << input.string() << endl;
        return true;
    }

    mExport    = true;
    mHeadless  = true;
    mUseV1     = !useV0;
    mModelName = name.empty() ? mInputPath.filename().string() : name;
    mOutputPath = out.empty() ? mInputPath : fs::path(out);

    if (countInputFrames(mInputPath) == 0) {
        console() << "[ERROR] no numbered OBJ frames (0.obj, 1.obj, ...) in " << mInputPath.string() << endl;
        return true;
    }

    try {
        loadModel(mInputPath);
    } catch (const std::exception& ex) {
        console() << "[ERROR] export failed: " << ex.what() << endl;
        return true;
    }

    if (mData.empty()) {
        console() << "[ERROR] export failed, see the log above" << endl;
        return true;
    }

    uint32_t totalFrames = 0;
    for (const auto& data : mData)
        totalFrames += data.z;

    if (mUseV1) {
        console() << "[EXPORTED] " << vat1OutputPath().string() << " - " << mData.size()
                  << (mData.size() == 1 ? " topology, " : " topologies, ")
                  << totalFrames << " frames" << endl;
    } else {
        console() << "[EXPORTED] " << mData.size() << " files (V0) -> " << vat0OutputDir().string() << endl;
    }
    return true;
}

void VATExporterApp::setup() {
    mModelLoaded    = false;
    mExport         = true;
    mHeadless       = false;
    mUseV1          = true;
    mTime           = 0.f;
    mSpeed          = 0.f;
    mInstanceCnt    = 4;
    mDirection      = 1.f;
    mVertCnt        = 0;
    mIndexCnt       = 0;
    mFrameCnt       = 0;
    mPrevVert       = 0;
    mPrevIdxCnt     = 0;
    mBundleIdx      = 0;

    // CLI mode runs once and exits before any GUI/GL work
    if (runFromCommandLine()) {
        quit();
        return;
    }

    ImGui::Initialize();

    mRenderShader = gl::GlslProg::create(loadResource(RENDER_VERT), loadResource(RENDER_FRAG));
#if 0
    {
        auto path = { getAssetPath("shader/render.vert"), getAssetPath("shader/render.frag") };
        FileWatcher::instance().watch(path, [this](const const WatchEvent& evt) {
            try {
                auto shader = gl::GlslProg::create(loadAsset("shader/render.vert"), loadAsset("shader/render.frag"));
                mRenderShader = shader;
            } catch (ci::Exception ex) {
                CI_LOG_EXCEPTION("Render glsl", ex);
            }
        });
    }
#endif

    mCam.setPerspective(45.f, getWindowAspectRatio(), .1f, 300.f);
    mCam.lookAt(10.f * vec3(2.f, 4.f, 2.f), vec3());
    mCamUi = CameraUi(&mCam, getWindow());

    gl::enableDepth();
    mPrevTime = (float)app::getElapsedSeconds();
}

void VATExporterApp::mouseDown( MouseEvent event ) {
}

void VATExporterApp::update() {
    float tt  = (float)app::getElapsedSeconds();
    mTime    += mSpeed * (tt - mPrevTime);
    mPrevTime = tt;

    {
        ImGui::ScopedWindow scpWin("Parameters");

        if (mModelLoaded) {
            ImGui::InputText("Model", &mModelName);

            string time = "Time: " + toString(glm::round(100.f * mTime) / 100.f);
            ImGui::Text(time.c_str());

            if (ImGui::Button("Reset")) mTime = 0.f;

            ImGui::SliderFloat("Speed", &mSpeed, 0.f, 10.f);
            ImGui::SliderFloat("Direction", &mDirection, -1.f, 1.f);

            static const vector<uint32_t> num = { 1,4,9,16,25,36 };
            static int idx = 1;
            if (ImGui::SliderInt("Instance Cnt", &idx, 0, num.size() - 1))
                mInstanceCnt = num[idx];
            static int bundle = mBundleIdx;
            if (ImGui::SliderInt("Frame", &bundle, 0, mData.size() - 1))
                mBundleIdx = bundle;

            if (ImGui::Button("Unload")) {
                mModelLoaded = false;
            }
        } else {
            static bool exportFile = true;
            static fs::path filepath;
            static int exportFmt = 1;   // 1 = V1 packed, 0 = V0 legacy
            static string prevDefaultName;

            ImGui::Checkbox("Export File", &exportFile);

            if (ImGui::Button("Load Input Obj File")) {
                CI_LOG_I("choose one of the obj frames to open");
                filepath = getOpenFilePath();
                if (fs::is_regular_file(filepath)) {
                    mInputPath = filepath.parent_path();
                    // default the model name to the input folder; keep user edits
                    string def = mInputPath.filename().string();
                    if (mModelName.empty() || mModelName == prevDefaultName)
                        mModelName = def, prevDefaultName = def;
                    // default the output next to the input
                    if (mOutputPath.empty())
                        mOutputPath = mInputPath;
                }
            }

            if (!filepath.empty()) {
                ImGui::Separator();
                ImGui::InputText("Model", &mModelName);

                if (exportFile) {
                    ImGui::RadioButton("V1 packed", &exportFmt, 1); ImGui::SameLine();
                    ImGui::RadioButton("V0 legacy", &exportFmt, 0);

                    string outLabel = "Output: " + (mOutputPath.empty() ? string("<none>") : mOutputPath.string());
                    ImGui::Text(outLabel.c_str());
                    ImGui::SameLine();
                    if (ImGui::Button("Choose Output...")) {
                        // the typed filename is kept: it becomes the .vat file (V1)
                        // or the output folder (V0) holding <name><N>.vat files
                        auto path = getSaveFilePath(mOutputPath, { "vat" });
                        if (!path.empty())
                            mOutputPath = path;
                    }
                }

                const bool ready = !exportFile || !mOutputPath.empty();
                if (ready && ImGui::Button("Execute")) {
                    mExport = exportFile;
                    mUseV1  = (exportFmt == 1);

                    if (countInputFrames(mInputPath) == 0) {
                        CI_LOG_E("no numbered OBJ frames (0.obj, 1.obj, ...) found in " << mInputPath.string());
                    } else {
                        if (mExport)
                            CI_LOG_I("Load from " << mInputPath << ", saving to " << mOutputPath
                                     << ", with model name " << mModelName
                                     << (mUseV1 ? " [V1 packed]" : " [V0 legacy]"));
                        else CI_LOG_I("Load existing vat files from " << mInputPath);

                        loadModel(mInputPath);
                        filepath.clear();
                    }
                }
            }
        }
    }
}

void VATExporterApp::draw() {
    gl::clear(Color(.2f, .2f, .2f));
    gl::color(1.f, 1.f, 1.f);

    if(mModelLoaded) {
        gl::ScopedVao         vaoScope   (mMeshVao[mBundleIdx]);
        gl::ScopedBuffer      bufferScope(mMeshIndex[mBundleIdx]);

        gl::ScopedTextureBind vertBuffer  (mVertBuffer[mBundleIdx]->getTarget(), mVertBuffer[mBundleIdx]->getId(), 0);
        gl::ScopedTextureBind normalBuffer(mNormalBuffer[mBundleIdx]->getTarget(), mNormalBuffer[mBundleIdx]->getId(), 1);
        
        gl::enableDepth(true);
        //gl::enableFaceCulling();
        gl::viewport(getWindowSize());
        gl::setMatrices(mCam);
        /*
        {
            gl::ScopedColor scpColor(Color(0,0,1));
            gl::ScopedGlslProg scpGlsl(gl::getStockShader(gl::ShaderDef().color()));
            gl::drawVector(vec3(0.), vec3(0.,0.,1.));
        }
         */
         //main render pass
        {
            //gl::cullFace(GL_BACK);
            gl::ScopedGlslProg   glslScope(mRenderShader);
            gl::ScopedBlendAlpha scpAlpha;
            mRenderShader->uniform("uCamPos",       mCam.getEyePoint());
            mRenderShader->uniform("uSize",         vec3(1.f));
            mRenderShader->uniform("uVertexBuffer", 0);
            mRenderShader->uniform("uNormalBuffer", 1);

            mRenderShader->uniform("uVertCnt",      (int)mData[mBundleIdx].y);
            mRenderShader->uniform("uFrameCnt",     (float)mData[mBundleIdx].z);
            mRenderShader->uniform("uBoundSize",    mBoundBox.getSize());
            mRenderShader->uniform("uSide",         (int)glm::sqrt(mInstanceCnt));
            mRenderShader->uniform("uTime",         mTime);

            gl::setDefaultShaderVars();
            gl::drawElementsInstanced(GL_TRIANGLES, mData[mBundleIdx].x, GL_UNSIGNED_INT, nullptr, mInstanceCnt);
        }

        gl::enableFaceCulling(false);
    }

    // plane - shadow receiver
    {
        gl::color(1.f, 1.f, 1.f);
        gl::drawCube(vec3(0.f, -5.f, 0.f), vec3(100.f, .01f, 100.f));
    }
}

CINDER_APP(VATExporterApp, RendererGl, [](App::Settings* settings) {
    settings->setWindowSize(1920, 1080);
    settings->setConsoleWindowEnabled();
})
