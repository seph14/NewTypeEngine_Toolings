# NewTypeEngine Toolings

Standalone content-creation tools for [NewTypeEngine](https://github.com/seph14/NewTypeEngine),
the open-source real-time path tracing engine. Each folder is a self-contained
Visual Studio project; everything the tools emit drops straight into the
engine's `assets/` folder in formats its loaders consume natively.

| Tool | Purpose | Emits |
|------|---------|-------|
| **VATExporter** | bake animated mesh sequences into vertex-animation textures | `.vat` |
| **FBXImporter** | convert FBX scenes into engine assets | `<stem>.json` + `.msh`/`.obj` + textures |
| **TetCage** | build + validate tetrahedral-cage deformation files | `.tetcage` |
| **Bundler** | pack project assets into the `Resources.h` / `Resources.rc` id system | `Resources.h` / `Resources.rc` |

## Requirements

- Windows 10/11, Visual Studio 2022, C++20.
  Bundler & VATExporter target the **v142** toolset, FBXImporter & TetCage the
  **v143** toolset — install the missing build tools via the VS installer if
  needed (or retarget the projects).
- [Cinder](https://github.com/seph14/Cinder) — please use the engine's fork:
  the tools link its static `Debug_MD` / `Release_MD` lib configs, which
  upstream cinder does not ship.
- Python 3 + `numpy` for TetCage's analysis scripts (`pip install numpy`).
- Nothing else — FBXImporter vendors [ufbx](https://github.com/ufbx/ufbx) and
  [meshoptimizer](https://github.com/zeux/meshoptimizer) under
  `FBXImporter/third_party/`.

## Building

Every tool carries its own solution: `Bundler/vc2019/Bundler.sln`,
`VATExporter/vc2019/VATExporter.sln`, `FBXImporter/vc2022/FBXImporter.sln`,
`TetCage/vc2022/tetcage.sln` (the folder name reflects the toolset era — all
build with current MSBuild):

```bash
msbuild FBXImporter\vc2022\FBXImporter.sln /p:Configuration=Release /p:Platform=x64
```

The tools resolve Cinder in two different ways — both expect the fork above:

- **FBXImporter / TetCage** — a `CinderRoot` user macro in the `.vcxproj`;
  point it at your Cinder checkout.
- **Bundler / VATExporter** — older projects with Cinder include/library paths
  hardcoded in the `.vcxproj`; edit them to match your checkout.

Per-tool `assets/` folders are gitignored — drop your own inputs there.

## GUI + CLI

All four tools are Cinder + ImGui apps: a windowed mode with panels,
drag-and-drop and an in-app log, plus a headless CLI passthrough — when
`--input` (or `--project`, for Bundler) is on the command line the app runs
the job and exits with a process exit code instead of opening the window.

## VATExporter

Bakes a vertex-animation sequence — a folder of contiguously numbered OBJ
frames (`0.obj`, `1.obj`, `2.obj`, …) — into the engine's `.vat` format
(vertex positions/normals packed into texture pages), with UVs repacked via
[xatlas](https://github.com/xatlas/xatlas) so every frame shares one layout.
The engine side consumes these through `scene/VATMesh` / `scene/VATLoader`,
and the TetCage tool reads the same files as cage-builder input.

```bash
VATExporter.exe --input frames_folder            # or a single frame file
                [--out out.vat] [--name my-anim] [--v0]
```

## FBXImporter

Converts an FBX scene into engine-ready assets: meshes are merged or kept as
a hierarchy, optionally optimized and simplified, units normalized, textures
extracted, and everything written to `<fbxDir>/<stem>_imported/` (or `--out`)
as `<stem>.json` + `<base>.msh` (engine binary) or `.obj`.

```bash
FBXImporter.exe --input model.fbx [--out dir] [--hierarchy|--merge]
                [--optimize|--no-optimize] [--simplify 0.5]
                [--format trimesh|obj] [--no-textures] [--raw-units]
                [--dry-run]
```

## TetCage

Stage-1 offline tetrahedral-cage builder + validator: dissects a rest-pose
mesh into per-tet "piece" micro-meshes plus the cage lattice they were cut
from, for the engine's massively-instanced `TetCageGeometry` deformation
pipeline. Includes a Gate-1 watertightness/area-conservation validator and
Python analysis scripts. See
[TetCage/README.md](TetCage/README.md) for the full manual.

```bash
TetCage\run.bat model.vat --res 16      # build (cl) + run
tet_cage_gui.exe --input model.vat      # headless via the GUI build
```

## Bundler

Manages the resource-id system engine and Cinder projects compile against
(`include/Resources.h` + `vc…/Resources.rc`). Point it at a project and it
scans the asset sources (GLSL, images, meshes, VATs, sounds, LUTs, plus
shader-bank and resource-update utilities), diffs them against the current
resource ids, and writes updated `Resources.h` / `Resources.rc`. Runs as a
dry-run report unless `--apply` is given.

```bash
Bundler.exe --project path\to\project           # scan + report only
Bundler.exe --project path\to\project --apply  # write Resources.h/.rc
```
