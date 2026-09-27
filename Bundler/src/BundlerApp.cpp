#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/CinderImGui.h"
#include "cinder/Utilities.h"
#include "cinder/Log.h"
#include <regex>

using namespace ci;
using namespace ci::app;
using namespace std;

class BundlerApp : public App {
public:
	enum class ResourceType {
		GLSL = 0, IMAGE = 1, MESH = 2, VAT = 3, SOUND = 4, UNKNOWN = 5,
		// shaderbank utilities
		BANK_GLSL_HEAD = 6, BANK_GLSL = 7, BANK_COMP = 8,
		// resource update utilities
		RES_GLSL_HEAD = 9, RES_GLSL = 10, RES_COMP = 11,
		// NewTypeEngine assets
		LUT = 12
	};

	fs::path mProjectPath;
	fs::path mVcDir;    // project folder holding Resources.rc (vc2019/vc2022/...)
	bool	 mBundleGlsl, mBundleTexture, mBundleMesh, mBundleVAT, mBundleLut, mBundleSound,
			 mUpdateResource, mDryRun, mRemoveAssetFileAfterCopy, mIncludeBlock, mBundleBank, mProcessed;
	string   mResourceContent, mRCContent;
	vector<string> mLog;
	int		 mStartResourceIdx, mCurrResourceIdx, mLines;
	uint32_t mChangedResources = 0, mAddedResources = 0, mChangedFiles = 0;

	//std::unordered_map<fs::path, std::string> mData;
	std::vector<std::tuple<fs::path, std::string, std::vector<std::string>,
										 std::vector<ivec2>, std::vector<ResourceType>>> mSourceFiles;

	void setup() override;
	void mouseDown( MouseEvent event ) override;
	void update() override;
	void draw() override;

	void process();
	static void splitString(const string& input, char delimiter, std::vector<string>& arr);
	std::string makeResource( const std::string& name, const std::string& path, const std::string& type );
	static const string Type2Str(const ResourceType& type);
	static const string Type2Res(const ResourceType& type);
	static const string GenResourceName(const std::string& path, const ResourceType& type);

	// Marks every byte that is real code ('x') vs comment/string (' ') so
	// example calls in documentation never match the scan loops.
	static string buildCodeMask(const string& content);
	static bool  isCode(const string& mask, size_t pos) { return pos < mask.size() && mask[pos] == 'x'; }
};

void BundlerApp::setup() {
	mStartResourceIdx = 0;
	mBundleGlsl		= true;
	mBundleTexture	= true;
	mBundleMesh		= true;
	mBundleVAT		= true;
	mBundleLut		= true;
	mBundleSound	= false;
	mDryRun			= true;
	mRemoveAssetFileAfterCopy = false;
	mProcessed		= false;
	mIncludeBlock	= true;
	mUpdateResource = true;
	mBundleBank		= false;

	// CLI mode: Bundler.exe --project <dir> [--apply] — run once and exit.
	// Build-script friendly; without --apply it stays a dry run like the GUI.
	const auto& args = getCommandLineArgs();
	fs::path cliProject;
	for (size_t i = 1; i < args.size(); ++i) {
		if (args[i] == "--project" && i + 1 < args.size()) cliProject = args[++i];
		else if (args[i] == "--apply") mDryRun = false;
	}
	if (!cliProject.empty()) {
		mProjectPath = cliProject;
		process();
		console() << (mDryRun ? "[DRY RUN] " : "[APPLIED] ")
				  << "Added Resources: " << mAddedResources
				  << ", Changed Source Files: " << mChangedFiles << endl;
		quit();
		return;
	}

	auto option = ImGui::Options();
	ImGuiStyle style = ImGuiStyle();
	style.ScaleAllSizes(1.25f);
	option.style(style);
	ImGui::Initialize(option);
}

void BundlerApp::mouseDown( MouseEvent event ) {
}

const string BundlerApp::Type2Str(const ResourceType& type) {
	static const vector<string> values = { "GLSL", "IMG", "MSH", "VAT", "SND", "UNKNOWN",
		"BANK_GLSL_HEAD", "BANK_GLSL", "BANK_COMP",
		"RES_GLSL_HEAD",  "RES_GLSL",  "RES_COMP",
		"LUT" };
	return values[(int)type];
}

const string BundlerApp::Type2Res(const ResourceType& type) {
	static const vector<string> values = { "GLSL", "IMG", "MSH", "VAT", "SND", "UNKNOWN",
		"GLSL", "GLSL", "GLSL",
		"GLSL", "GLSL", "GLSL",
		"LUT" };
	return values[(int)type];
}

string BundlerApp::buildCodeMask(const string& content) {
	string mask(content.size(), 'x');
	enum State { CODE, LINE_COMMENT, BLOCK_COMMENT, STRING, CHAR_LITERAL };
	State state = CODE;
	for (size_t i = 0; i < content.size(); ++i) {
		char c = content[i];
		char n = (i + 1 < content.size()) ? content[i + 1] : '\0';
		switch (state) {
		case CODE:
			if (c == '/' && n == '/')			{ mask[i] = mask[i + 1] = ' '; state = LINE_COMMENT; ++i; }
			else if (c == '/' && n == '*')		{ mask[i] = mask[i + 1] = ' '; state = BLOCK_COMMENT; ++i; }
			else if (c == '"')					{ mask[i] = ' '; state = STRING; }
			else if (c == '\'')					{ mask[i] = ' '; state = CHAR_LITERAL; }
			break;
		case LINE_COMMENT:
			mask[i] = ' ';
			if (c == '\n') state = CODE;
			break;
		case BLOCK_COMMENT:
			mask[i] = ' ';
			if (c == '*' && n == '/') { mask[i + 1] = ' '; state = CODE; ++i; }
			break;
		case STRING:
			mask[i] = ' ';
			if (c == '\\' && n != '\0')		{ mask[i + 1] = ' '; ++i; }
			else if (c == '"')					state = CODE;
			else if (c == '\n')					state = CODE;	// unterminated safety
			break;
		case CHAR_LITERAL:
			mask[i] = ' ';
			if (c == '\\' && n != '\0')		{ mask[i + 1] = ' '; ++i; }
			else if (c == '\'')					state = CODE;
			else if (c == '\n')					state = CODE;	// unterminated safety
			break;
		}
	}
	return mask;
}

const string BundlerApp::GenResourceName(const std::string& path, const ResourceType& type) {
	//"shaders/files/render.vert"
	vector<string> tokens;
	splitString( path, '/', tokens );
	string resname = "";

	auto name = tokens.back();
	tokens.pop_back();
	for (auto& str : tokens) {
		if (str == "shader" || str == "shaders") continue;
		std::transform(str.begin(), str.end(), str.begin(), ::toupper);
		resname += str + "_";
	}
	tokens.clear();
	splitString(name, '.', tokens);

	string filename = tokens[0];
	std::transform(filename.begin(), filename.end(), filename.begin(), ::toupper);
	if (type != ResourceType::GLSL && type != ResourceType::BANK_GLSL && type != ResourceType::RES_GLSL &&
		type != ResourceType::BANK_GLSL_HEAD && type != ResourceType::BANK_COMP &&
		type != ResourceType::RES_GLSL_HEAD && type != ResourceType::RES_COMP) {
		resname += filename + "_" + Type2Str(type);
	} else {
		string filetype = tokens[1];
		std::transform(filetype.begin(), filetype.end(), filetype.begin(), ::toupper);
		resname += filename + "_" + filetype;
	}

	return resname;
}

std::string BundlerApp::makeResource(const std::string& name, const std::string& path, const std::string& type) {
	return "#define " + name + " CINDER_RESOURCE( ../resources/, " + path + ", " + toString(mCurrResourceIdx++) +  ", " + type + " )";
}

void BundlerApp::splitString(const string& input, char delimiter, std::vector<string>& arr) {
	istringstream stream(input);
	string token;
	while (getline(stream, token, delimiter)) {
		if(token.size() > 0) arr.push_back(token);
	}
}

void BundlerApp::process() {
	mSourceFiles.clear();

	// locate the vc project folder carrying Resources.rc (vc2019/vc2022/...);
	// pick the lexicographically highest version when several exist
	mVcDir = mProjectPath / "vc2019";
	for (const auto& entry : fs::directory_iterator{ mProjectPath }) {
		if (!entry.is_directory()) continue;
		const auto name = entry.path().filename().string();
		if (name.compare(0, 2, "vc") == 0 && name > mVcDir.filename().string()
			&& fs::exists(entry.path() / "Resources.rc")) {
			mVcDir = entry.path();
		}
	}

	mResourceContent = loadString(loadFile(mProjectPath / "include" / "Resources.h"));
	const auto rcPath = mVcDir / "Resources.rc";
	mRCContent		 = fs::exists(rcPath) ? loadString(loadFile(rcPath))
										 : "#include \"../include/Resources.h\"\r\n";
	mChangedResources= 0;
	mAddedResources  = 0;
	mChangedFiles	 = 0;
	mLines			 = 0;
	mLog.clear();

	unordered_map<string, string> resourceFilePath;

	// parse resource file to get the largest resource ID
	{
		mStartResourceIdx = 0;
		vector<string> rcLines, tokens, resourceNames;
		string cacheLine;
		splitString(mResourceContent, '\n', rcLines);
		for (auto& line : rcLines) {
			splitString(line, ',', tokens);
			if (tokens.size() >= 4) {
				// cache resource file path, so we can update later
				if (mUpdateResource) {
					// split first token to get the name
					splitString(tokens[0], ' ', resourceNames);
					//cacheLine = resourceNames
					cacheLine = resourceNames[1];
					resourceNames.clear();
					splitString(cacheLine, '\t', resourceNames);
					// 2nd token is the file path
					resourceFilePath[resourceNames[0]] = tokens[1];
					resourceNames.clear();
				}

				// check token
				auto lastCtn = tokens[tokens.size() - 2];

				try {
					mStartResourceIdx = glm::max(mStartResourceIdx, fromString<int>(lastCtn));
				} catch (const ci::Exception& ex) {
					CI_LOG_EXCEPTION("Integer parse failed", ex);
				}
			}
			tokens.clear();
		}
		CI_LOG_I("Max Resource ID: " << mStartResourceIdx);
		// keep the >= 128 convention: template-era projects hand-seed 128+ and
		// ShaderBank treats ids below 128 as the "no resource" sentinel
		mStartResourceIdx = glm::max(mStartResourceIdx, 128);
		mCurrResourceIdx = mStartResourceIdx + 1;
	}

	if (!fs::exists(mProjectPath / "src")) {
		CI_LOG_E("No src/ folder found under " << mProjectPath.string());
		return;
	}

	// recursively collect project sources (.cpp/.h); NewTypeEngine-style
	// projects nest their code under src/<module>/...
	for (const auto& entry : fs::recursive_directory_iterator{ mProjectPath / "src" }) {
		if (entry.is_directory()) continue;
		auto ext = entry.path().extension().string();
		std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
		if (ext != ".cpp" && ext != ".h" && ext != ".hpp") continue;
		// the scaffolder copies this file into new projects — keep it pristine
		if (entry.path().filename().string() == "Template.cpp") continue;
		mSourceFiles.push_back({ entry.path(), loadString(loadFile(entry.path())), {}, {}, {} });
	}

	if (mIncludeBlock && fs::exists(mProjectPath / "blocks" / "SolidJellyfishRenderLib")) {
		for (const auto& entry : fs::directory_iterator{ mProjectPath / "blocks"  / "SolidJellyfishRenderLib" / "src" / "Effects"}) {
			if (!entry.is_directory())
				mSourceFiles.push_back({ entry.path(), loadString(loadFile(entry.path())), {}, {}, {} });
		}
	}

	// asset check
	static const string loadReg = "loadAsset(";
	static const string glslReg = "ShaderBank::watchGlsl(";
	static const string compReg = "ShaderBank::watchComp(";

	// resource check
	static const string glslResReg = "ShaderBank::loadGlsl(";
	static const string compResReg = "ShaderBank::loadComp(";

	// step over spaces/tabs/newlines starting at p
	static const auto skipWs = [](const string& s, size_t p) {
		while (p < s.size() && (s[p] == ' ' || s[p] == '\t' || s[p] == '\r' || s[p] == '\n')) ++p;
		return p;
	};

	for (auto& srcFile : mSourceFiles) {
		size_t  pos = 0, nchars = 0, echars = 0, b, e;

		string content				= std::get<1>(srcFile);
		vector<string>& files		= std::get<2>(srcFile);
		std::vector<ivec2>& range	= std::get<3>(srcFile);
		auto& type					= std::get<4>(srcFile);
		const auto codeMask			= buildCodeMask(content);

		app::console() << "    " << std::get<0>(srcFile).string() << endl;
		while ((nchars = content.find(loadReg, pos)) != std::string::npos) {
			// Multi-argument safe: only the `loadAsset("literal"` prefix is
			// replaced (up to the closing quote) so trailing arguments survive —
			// loadAsset("x.png", device, ...) becomes loadResource(RES_NAME, device, ...).
			// Line-broken calls are allowed; dynamic (quoteless) calls and
			// matches inside comments/strings are skipped.
			const size_t argStart = skipWs(content, nchars + loadReg.size());
			if (!isCode(codeMask, nchars) || argStart >= content.size() || content[argStart] != '"') {
				pos = nchars + 1;
				continue;
			}
			e = content.find("\"", argStart + 1);
			// closing quote must be followed (after optional whitespace) by ','
			// or ')' so concatenated literals like "a" + x are rejected
			const size_t after = (e == string::npos) ? 0 : skipWs(content, e + 1);
			const bool cleanLiteral = e != string::npos && after < content.size()
				&& (content[after] == ',' || content[after] == ')');
			if (!cleanLiteral) {
				pos = nchars + 1;
				continue;
			}

			echars		 = e + 1;   // span end: just past the closing quote
			auto loadStr = content.substr(nchars, echars - nchars);
			auto src	 = content.substr(argStart + 1, e - argStart - 1);

			// ignore GLSL ES for iOS
			if (src.find("shaders_es") == string::npos && src.find("shader_es") == string::npos) {
				range.push_back(ivec2(nchars, echars));
				files.push_back(src);

				auto srcLower = src;
				std::transform(srcLower.begin(), srcLower.end(), srcLower.begin(), ::tolower);
				if (srcLower.find(".glsl") != string::npos || srcLower.find(".vert") != string::npos ||
					srcLower.find(".frag") != string::npos || srcLower.find(".comp") != string::npos || srcLower.find(".geom") != string::npos)
					type.push_back(ResourceType::GLSL);
				else if (srcLower.find(".jpg") != string::npos || srcLower.find(".jpeg") != string::npos ||
					srcLower.find(".png") != string::npos || srcLower.find(".dds") != string::npos ||
					srcLower.find(".hdr") != string::npos || srcLower.find(".hdri") != string::npos ||
					srcLower.find(".exr") != string::npos || srcLower.find(".tga") != string::npos ||
					srcLower.find(".bmp") != string::npos || srcLower.find(".tif") != string::npos)
					type.push_back(ResourceType::IMAGE);
				else if (srcLower.find(".obj") != string::npos || srcLower.find(".msh") != string::npos)
					type.push_back(ResourceType::MESH);
				else if (srcLower.find(".vat") != string::npos || srcLower.find(".mph") != string::npos)
					type.push_back(ResourceType::VAT);
				else if (srcLower.find(".cube") != string::npos || srcLower.find(".3dl") != string::npos)
					type.push_back(ResourceType::LUT);
				else if (srcLower.find(".wav") != string::npos || srcLower.find(".mp3") != string::npos ||
					srcLower.find(".ogg") != string::npos || srcLower.find(".flac") != string::npos ||
					srcLower.find(".aiff") != string::npos)
					type.push_back(ResourceType::SOUND);
				else type.push_back(ResourceType::UNKNOWN);

				app::console() << loadStr << " - " << src << "::" << Type2Str(type.back()) << endl;
			} else
				app::console() << "ignore GLSL ES --> " << loadStr << endl;

			pos = echars;
		}

		// check watchGlsl
		pos = 0; nchars = 0; echars = 0;
		while ((nchars = content.find(glslReg, pos)) != std::string::npos) {
			if (!isCode(codeMask, nchars)) { pos = nchars + 1; continue; }
			echars			= content.find("{", nchars) - 1;
			auto loadStr	= content.substr(nchars, echars - nchars);
			auto glsl		= split(loadStr, ',');
			bool firstInst	= true;
			echars			= nchars + glsl[0].size();
			for(auto& file : glsl) {
				if (file.size() == 0) continue;
				if (!firstInst) {
					nchars  = echars;
					echars += file.size() + 2;
				}

				b = file.find_first_of("\"") + 1;
				e = file.find_last_of("\"");
				if (e <= b) continue;
				
				//app::console() << file << endl;
				range.push_back(ivec2(nchars, echars));
				auto src = file.substr(b, e - b);
				files.push_back("shaders/" + src);
				
				if (firstInst) {
					firstInst = false;
					type.push_back(ResourceType::BANK_GLSL_HEAD);
				} else
					type.push_back(ResourceType::BANK_GLSL);
				app::console() << file << " - " << src << "::" << Type2Str(type.back()) << endl;
				
			} 

			pos = echars;
		}

		// check loadGlsl
		pos = 0; nchars = 0; echars = 0;
		while ((nchars = content.find(glslResReg, pos)) != std::string::npos) {
			if (!isCode(codeMask, nchars)) { pos = nchars + 1; continue; }
			echars			= content.find("{", nchars) - 1;
			auto loadStr	= content.substr(nchars, echars - nchars);
			auto glsl		= split(loadStr, ',');
			bool firstInst	= true;
			echars			= nchars + glsl[0].size();
			for (auto& file : glsl) {
				if (file.size() == 0) continue;
				if (!firstInst) {
					nchars  = echars;
					echars += file.size() + 2;
				}

				b = file.find_first_of("(") + 1;
				e = file.find_last_of(")");
				if (e <= b) continue;

				range.push_back(ivec2(nchars, echars));
				auto src = file.substr(b, e - b);
				if (resourceFilePath.find(src) != resourceFilePath.end()) {
					files.push_back(trim(resourceFilePath[src]));

					if (firstInst) {
						firstInst = false;
						type.push_back(ResourceType::RES_GLSL_HEAD);
					} else
						type.push_back(ResourceType::RES_GLSL);
					app::console() << file << " - " << src << "::" << Type2Str(type.back()) << endl;
				} else {
					CI_LOG_W("Requested resource " << src << " not found, skip");
				}
			}

			pos = echars;
		}

		// check watchComp
		pos = 0; nchars = 0; echars = 0;
		while ((nchars = content.find(compReg, pos)) != std::string::npos) {
			if (!isCode(codeMask, nchars)) { pos = nchars + 1; continue; }
			echars		 = content.find("{", nchars) - 1;
			auto loadStr = content.substr(nchars, echars - nchars);
			
			b = loadStr.find_first_of("\"") + 1;
			e = loadStr.find_last_of("\"");
				
			range.push_back(ivec2(nchars, echars));
			auto src = loadStr.substr(b, e - b);
			files.push_back("shaders/" + src);
			type.push_back(ResourceType::BANK_COMP);
			app::console() << loadStr << " - " << src << "::" << Type2Str(type.back()) << endl;

			pos = echars;
		}

		// check loadComp
		pos = 0; nchars = 0; echars = 0;
		while ((nchars = content.find(compResReg, pos)) != std::string::npos) {
			if (!isCode(codeMask, nchars)) { pos = nchars + 1; continue; }
			echars		 = content.find("{", nchars) - 1;
			auto loadStr = content.substr(nchars, echars - nchars);

			b = loadStr.find_first_of("(") + 1;
			e = loadStr.find_last_of(",");

			range.push_back(ivec2(nchars, echars));
			auto src = loadStr.substr(b, e - b);
			if (resourceFilePath.find(src) != resourceFilePath.end()) {
				files.push_back	(trim(resourceFilePath[src]));
				type.push_back	(ResourceType::RES_COMP);
				app::console() << loadStr << " - " << src << "::" << Type2Str(type.back()) << endl;
			} else {
				CI_LOG_W("Requested resource " << src << " not found, skip");
			}

			pos = echars;
		}

		// only handles GLSL now, may need to add textures later
		if (mBundleBank) {
			pos		= 0;
			nchars	= 0;
			echars	= 0;
			vector<string> tokens;

			while ((nchars = content.find(glslReg, pos)) != std::string::npos) {
				echars		 = content.find(")", nchars) + 1;
				auto loadStr = content.substr(nchars, echars - nchars);
				splitString(loadStr, '"', tokens);

				uint32_t offset = nchars + tokens[0].size() + 1;
				for (uint32_t idx = 1; idx < tokens.size() - 1; idx+=2) {
					auto src   = "shaders/" + tokens[idx];
					auto asset = content.substr(offset - 1, tokens[idx].size() + 2);
					range.push_back(ivec2(offset - 1, offset + 1 + tokens[idx].size()));
					files.push_back(src);
					type.push_back (ResourceType::GLSL);

					offset	  += tokens[idx].size() + tokens[idx + 1].size() + 2;
					app::console() << asset << " - " << src << "::BankGLSL" << endl;
				}
				tokens.clear();
				pos = echars;
			}

			pos		= 0;
			nchars	= 0;
			echars	= 0;
			tokens.clear();

			while ((nchars = content.find(compReg, pos)) != std::string::npos) {
				echars		 = content.find(")", nchars) + 1;
				auto loadStr = content.substr(nchars, echars - nchars);
				splitString(loadStr, '"', tokens);

				uint32_t offset = nchars + tokens[0].size() + 1;
				for (uint32_t idx = 1; idx < tokens.size() - 1; idx += 2) {
					auto src	= "shaders/" + tokens[idx];
					auto asset	= content.substr(offset - 1, tokens[idx].size() + 2);
					range.push_back(ivec2(offset - 1, offset + 1 + tokens[idx].size()));
					files.push_back(src);
					type.push_back(ResourceType::GLSL);

					offset += tokens[idx].size() + tokens[idx + 1].size() + 2;
					app::console() << asset << " - " << src << "::BankComp" << endl;
				}
				tokens.clear();
				pos = echars;
			}
		}
	}

	gl::ShaderPreprocessor glslProcessor;
	glslProcessor.addSearchDirectory(mProjectPath / "assets");
	string currLog = "";

	// append to resources
	for (auto& srcFile : mSourceFiles) {
		uint32_t idx = 0;
		mChangedResources = 0;

		string& content				= std::get<1>(srcFile);
		vector<string>& files		= std::get<2>(srcFile);
		std::vector<ivec2>& ranges	= std::get<3>(srcFile);
		auto& types					= std::get<4>(srcFile);
		
		vector<string> assetStrData;
		for (auto& range : ranges) 
			assetStrData.push_back(content.substr(range.x, range.y - range.x));
		
		currLog += std::get<0>(srcFile).filename().string() + "--->\r\n";
		mLines++;
		if (mLines > 48) {
			mLog.push_back(currLog);
			mLines  = 0;
			currLog = "";
		}

		for (; idx < types.size(); idx++) {
			auto& type  = types[idx];
			auto& range = ranges[idx];
			if (mBundleGlsl && type == ResourceType::GLSL) {
				try {
					auto glsl	 = glslProcessor.parse(mProjectPath / "assets" / files[idx]);
					auto resname = GenResourceName(files[idx], type);
					// if duplicate, ignore
					if (mResourceContent.find(resname) == string::npos) {
						// add to resource bundling
						mResourceContent += makeResource(resname, files[idx], Type2Res(type)) + "\r\n";
						mRCContent		 += resname + "\r\n";
						mAddedResources ++;
					
						currLog += "    " + files[idx] + "\r\n";
						mLines++;
						if (mLines > 48) {
							mLog.push_back(currLog);
							mLines  = 0;
							currLog = "";
						}
					} 

					// replace source file — no closing paren: the span ends at
					// the literal's closing quote, the original text supplies it
					auto assetStr	= assetStrData[idx];
					auto pt			= content.find(assetStr);
					if (pt != string::npos) {
						auto resourceStr = "loadResource(" + resname;
						content			 = content.replace(content.begin() + pt, content.begin() + pt + assetStr.length(),
														   resourceStr.begin(), resourceStr.end());
					}

					if (!mDryRun) writeString(mProjectPath / "resources" / files[idx], glsl);
					mChangedResources++;
					
					//app::console() << glsl << endl;
				} catch (const ci::Exception& ex) {
					CI_LOG_EXCEPTION("Glsl parse failed: " << files[idx], ex);
				}
			} else if ((mBundleGlsl     && (type == ResourceType::BANK_GLSL_HEAD || type == ResourceType::BANK_COMP)) || 
					   (mUpdateResource && (type == ResourceType::RES_GLSL_HEAD  || type == ResourceType::RES_COMP ))) {
				try {
					auto glsl	 = glslProcessor.parse(mProjectPath / "assets" / files[idx]);
					
					if (type == ResourceType::BANK_GLSL_HEAD || type == ResourceType::BANK_COMP) {
						auto resname = GenResourceName(files[idx], type);
						// if duplicate, ignore
						if (mResourceContent.find(resname) == string::npos) {
							// add to resource bundling
							mResourceContent += makeResource(resname, files[idx], Type2Res(type)) + "\r\n";
							mRCContent		 += resname + "\r\n";
							mAddedResources++;

							currLog += "    " + files[idx] + "\r\n";
							mLines++;
							if (mLines > 48) {
								mLog.push_back(currLog);
								mLines  = 0;
								currLog = "";
							}
						}

						// replace source file
						auto assetStr = assetStrData[idx];
						auto pt = content.find(assetStr);
						if (pt != string::npos) {
							auto resourceStr = (type == ResourceType::BANK_GLSL_HEAD ? "ShaderBank::loadGlsl(" : "ShaderBank::loadComp(") + resname + ",";
							content = content.replace(content.begin() + pt, content.begin() + pt + assetStr.length(),
								resourceStr.begin(), resourceStr.end());
						}
					}

					if (!mDryRun) writeString(mProjectPath / "resources" / files[idx], glsl);
					mChangedResources++;
					//app::console() << glsl << endl;
				} catch (const ci::Exception& ex) {
					CI_LOG_EXCEPTION("Glsl parse failed: " << files[idx], ex);
				}
			} else if (	(mBundleGlsl && type == ResourceType::BANK_GLSL) || 
						(mUpdateResource && type == ResourceType::RES_GLSL)) {
				try {
					auto glsl = glslProcessor.parse(mProjectPath / "assets" / files[idx]);
					if (type == ResourceType::BANK_GLSL) {
						auto resname = GenResourceName(files[idx], type);

						// if duplicate, ignore
						if (mResourceContent.find(resname) == string::npos) {
							// add to resource bundling
							mResourceContent += makeResource(resname, files[idx], Type2Res(type)) + "\r\n";
							mRCContent		 += resname + "\r\n";
							mAddedResources++;

							currLog += "    " + files[idx] + "\r\n";
							mLines++;
							if (mLines > 48) {
								mLog.push_back(currLog);
								mLines = 0;
								currLog = "";
							}
						}

						// replace source file
						auto assetStr = assetStrData[idx];
						auto pt = content.find(assetStr);
						if (pt != string::npos) {
							auto resourceStr = resname + ",";
							content = content.replace(content.begin() + pt, content.begin() + pt + assetStr.length(),
								resourceStr.begin(), resourceStr.end());
						}
					}

					if (!mDryRun) writeString(mProjectPath / "resources" / files[idx], glsl);
					mChangedResources++;
					//app::console() << glsl << endl;
				} catch (const ci::Exception& ex) {
					CI_LOG_EXCEPTION("Glsl parse failed: " << files[idx], ex);
				}
			} else if (	(mBundleTexture && type == ResourceType::IMAGE) ||
						(mBundleMesh    && type == ResourceType::MESH) ||
						(mBundleVAT     && type == ResourceType::VAT) ||
						(mBundleLut     && type == ResourceType::LUT) ||
						(mBundleSound   && type == ResourceType::SOUND)) {
				auto resname = GenResourceName(files[idx], type);
				// if duplicate, ignore
				if (mResourceContent.find(resname) == string::npos) {
					// add to resource bundling
					mResourceContent += makeResource(resname, files[idx], Type2Res(type)) + "\r\n";
					mRCContent		 += resname + "\r\n";
					mAddedResources++;

					currLog += "    " + files[idx] + "\r\n";
					mLines++;
					if (mLines > 48) {
						mLog.push_back(currLog);
						mLines  = 0;
						currLog = "";
					}
				}

				// replace source file — no closing paren, trailing args survive
				auto assetStr	= assetStrData[idx];
				auto pt			= content.find(assetStr);
				if (pt != string::npos) {
					auto resourceStr = "loadResource(" + resname;
					content			 = content.replace(content.begin() + pt, content.begin() + pt + assetStr.length(),
													   resourceStr.begin(), resourceStr.end());
				}

				if (!mDryRun) {
					try {
						auto targetPath = mProjectPath / "resources" / files[idx];
						auto parentPath = targetPath.parent_path();
						auto srcPath	= mProjectPath / "assets" / files[idx];
						if (!fs::exists(parentPath)) fs::create_directory(parentPath);
						fs::copy_file(mProjectPath / "assets" / files[idx], targetPath, fs::copy_options::overwrite_existing);
						if (mRemoveAssetFileAfterCopy) fs::remove(srcPath);
					} catch (const filesystem::filesystem_error& ex) {
						CI_LOG_EXCEPTION("File system error: " << files[idx], ex);
					} catch (const Exception& ex) {
						CI_LOG_EXCEPTION("File copy failed: " << files[idx], ex);
					}
				}
				
				mChangedResources++;
			} 
		}

		mChangedFiles += (mChangedResources > 0) ? 1 : 0;
		if (!mDryRun && mChangedResources > 0) {
			// rewritten code references the generated defines — make sure the
			// file can see Resources.h. Insert before the FIRST #include: the
			// last one may sit inside a #if block and get compiled out.
			if (content.find("#include \"Resources.h\"") == string::npos) {
				auto firstInc = content.find("#include");
				if (firstInc != string::npos) {
					auto lineStart = content.rfind("\n", firstInc);
					content.insert(lineStart == string::npos ? firstInc : lineStart + 1,
								   "#include \"Resources.h\"\r\n");
				} else
					content.insert(0, "#include \"Resources.h\"\r\n");
			}
			auto filepath = std::get<0>(srcFile);
			writeString(filepath, content);
		} else if (mChangedResources > 0) {
			//app::console() << "updated file: " << content << endl;
		}
	}

	if(!currLog.empty()) mLog.push_back(currLog);
	mProcessed = true;
	//app::console() << mResourceContent << endl;
	if (!mDryRun) {
		writeString(mProjectPath / "include" / "Resources.h", mResourceContent);
		writeString(mVcDir / "Resources.rc", mRCContent);
	}
}

void BundlerApp::update() {
	{
		ImGui::ScopedWindow scpWin("Controls", true);
		ImGui::SetWindowSize(ivec2(720,780));
		ImGui::SetWindowPos (ivec2(10, 10));
		
		string path = "Project Path: " + mProjectPath.string();
		ImGui::Text(path.c_str());

		if (ImGui::Button("Open Project")) {
			auto path = app::getOpenFilePath();
			if (!path.empty()) 
				mProjectPath = path.parent_path().parent_path();
		}

		ImGui::Dummy(ImVec2(0, 5));
		ImGui::Checkbox("Bundle Glsl",		&mBundleGlsl);
		ImGui::Checkbox("Bundle Texture",	&mBundleTexture);
		ImGui::Checkbox("Bundle Mesh",		&mBundleMesh);
		ImGui::Checkbox("Bundle VAT",		&mBundleVAT);
		ImGui::Checkbox("Bundle Lut",		&mBundleLut);
		ImGui::Checkbox("Bundle Sound",		&mBundleSound);
		ImGui::Checkbox("Bundle Bank",		&mBundleBank);
		ImGui::Checkbox("Update Resource",  &mUpdateResource);
		ImGui::Checkbox("Include Block",	&mIncludeBlock);
		ImGui::Checkbox("Dry Run",			&mDryRun);
		ImGui::Checkbox("Remove After Copy",&mRemoveAssetFileAfterCopy);

		ImGui::Dummy(ImVec2(0, 5));
		if (!mProjectPath.empty() && ImGui::Button("Process")) process();

		ImGui::Dummy(ImVec2(0, 5));
		if (mProcessed) {
			string rc = "Added Resources: " + toString(mAddedResources);
			ImGui::Text(rc.c_str());
			string rs = "Changes Source Files: " + toString(mChangedFiles);
			ImGui::Text(rs.c_str());
			for(auto& log : mLog) ImGui::Text(log.c_str());
		}
	}
}

void BundlerApp::draw() {
	gl::clear( Color( 0, 0, 0 ) ); 
}

CINDER_APP(BundlerApp, RendererGl, [](App::Settings* settings) {
	settings->setWindowSize	(740,800);
	settings->setResizable	(false);
	settings->setConsoleWindowEnabled();
})
