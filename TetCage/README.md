# TetCage — Stage-1 offline tetrahedral-cage builder + validator

Implements the preprocessing half of *"Ray Tracing Massive Amounts of Animated
Geometry"* (Gruen et al., HPG 2026) for [NewTypeEngine](https://github.com/seph14/NewTypeEngine):
rest-pose mesh → voxel grid → 6 conforming tets/voxel → clip source triangles
into per-tet "pieces" → `.tetcage` binary consumed by the engine's
`scene/TetCageGeometry` (`TetCageScene`, `--scene tetcage`).

Same tooling family as the VAT exporter, FBX importer and Bundler. The
`.tetcage` binary layout is written by `src/tet_cage_core.h` and documented
from the consumer side in the engine repo's
[`docs/tetcage.md`](https://github.com/seph14/NewTypeEngine/blob/master/docs/tetcage.md).

## CLI builder (`src/tet_cage.cpp`)

Single file, C++20, no engine/Cinder dependency. Core logic lives in
`src/tet_cage_core.h` (shared with the GUI); `src/tet_cage.cpp` is the CLI driver.

Build (MSVC — uses `vswhere` to locate Visual Studio):

```bat
TetCage\run.bat                        :: build + run (first asset in TetCage\assets)
TetCage\run.bat model.vat --topo 5     :: build + run with args
```

or directly:

```bat
cl /std:c++20 /EHsc /O2 /utf-8 /DNOMINMAX src\tet_cage.cpp /Fe:tet_cage.exe
```

Usage:

```
tet_cage <input.(obj|vat)> [--topo i|-1] [--frame i] [--res r]
         [--out path] [--objout path] [--samples n] [--seed s]
```

| option | default | meaning |
|---|---|---|
| `--topo` | `0` | VAT topology index; `-1` = all topologies (writes `<stem>_tN.tetcage` each) |
| `--frame` | `0` | rest-pose frame for VAT inputs |
| `--res` | `16` | cage resolution: voxels along the longest axis (`h = maxExtent/res`) |
| `--out` | derived | output `.tetcage` path |
| `--objout` | — | also dump the piece soup as OBJ (per-tet objects) for inspection |
| `--samples` | `200000` | Gate-1 two-way surface sample count |
| `--seed` | fixed | RNG seed (deterministic builds) |

Exit codes: `0` all topologies passed Gate 1, `1` gate failure, `2` usage /
load error.

Outputs:

- `.tetcage` v1 — binary format (header, cage verts, tet table with face
  neighbors, piece verts with normal/UV, piece tris). Loaded by the engine's
  `scene/TetCageGeometry`.
- `--objout` — human-inspectable piece soup (`o tet_N` object per piece).

Gate 1 (run automatically per topology): area conservation, cross-face chord
pairing, two-way surface sampling. A PASS here is the precondition for
engine-side consumption.

Typical runs:

```bat
tet_cage.exe assets\model.vat --res 16
tet_cage.exe model.vat --topo -1            :: all topologies, Gate-1 sweep
tet_cage.exe model.vat --topo 8 --res 8 --objout t8_pieces.obj
```

Cage-resolution scaling (ginkgo leaf, all Gate-1 PASS): tets and piece
triangles grow ~quadratically / superlinearly with `--res` — res 8 → 378 tets
/ 1965 piece tris, res 16 → 1542 / 4899, res 24 → 3489 / 8811. Choose the
largest res whose deformed A/B error stays in tolerance; the paper uses 6–25
voxels per axis.

## GUI (`src/tet_cage_gui.cpp`)

Cinder + ImGui front-end, same app shell as the FBXImporter tool: left panel
for input/parameters/stats/log, right viewport with source mesh, piece soup
(optionally per-tet colored), cage wireframe and welded-boundary rims;
drag-and-drop or `Open...` for inputs; `Save .tetcage` / `Dump OBJ` for
outputs.

Build (MSVC; `CinderRoot` is a project macro pointing at your Cinder
checkout — built with the `Debug_MD`/`Release_MD` static configs):

```bat
msbuild TetCage\vc2022\tet_cage_gui.vcxproj /p:Configuration=Release /p:Platform=x64
```

CLI passthrough (headless, FBXImporter pattern — detects `--input`):

```
tet_cage_gui.exe --input model.vat [--topo i] [--frame i] [--res r] [--out p] [--objout p]
```

Note: the rim overlay flags *every* welded-boundary edge of the soup,
including the source mesh's own open boundary (e.g. a leaf outline) — use
`scripts/watertight_welded.py` before reading rims as holes.

## Analysis helpers (Python, `scripts/`)

Require `numpy` (`pip install numpy`).

- `watertight_welded.py <cage.tetcage> [<source.vat> <topo>]` —
  **the authoritative watertightness check**: position-welds the piece soup
  globally by position, counts edges by their number of triangle uses across
  the whole soup, and classifies single-use (welded-boundary) edges as
  on-source-boundary (legit, e.g. leaf outlines) vs interior (genuine
  unpaired seam / crack candidate). For interior rims it also measures the
  distance to the nearest edge of any other piece (T-junction overlap vs
  hole). Healthy ginkgo output: 313 legit + 13 micro rims, 0 holes.
- `trace_source_edge.py <cage.tetcage> <source.vat> <topo>` — for a chosen
  source edge (or the worst rims automatically), collects every piece edge
  lying on it, orders them along the edge, and reports per-tet coverage:
  contributing tets, endpoint mismatches between consecutive sub-segments,
  and uncovered gaps. Also measures the crack-width distribution
  (endpoint-Hausdorff) across all rims.

## Engine-side consumption

`.tetcage` → `--scene tetcage` in NewTypeEngine (`src/tests/TetCageScene.cpp`)
via `scene/TetCageGeometry`; see the engine repo's
[`docs/tetcage.md`](https://github.com/seph14/NewTypeEngine/blob/master/docs/tetcage.md)
for the full pipeline (GPU cage deformation, TetSolve, TLAS refit) and scene
knobs.
