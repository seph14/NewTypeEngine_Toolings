#!/usr/bin/env python3
"""Correct watertightness analysis: position-welded edge multiplicity.

Counts every edge by its number of TRIANGLE USES across the whole soup
(no per-piece set dedupe — that misclassifies edges shared by two triangles
of the same piece), after welding vertices globally by position.

A welded-boundary edge (exactly one use) is then classified:
  source-boundary - lies on the source mesh's own open boundary (legit)
  interior        - genuine unpaired seam (crack candidate)
Also reports, for interior welded boundaries, the distance to the nearest
edge of any OTHER piece (coverage test: is the surface continued by a
neighbor piece, i.e., a T-junction overlap rather than a hole?).
"""
import struct
import sys
from collections import defaultdict

import numpy as np


def load_vat(path, topo=0, frame=0):
    data = open(path, "rb").read()
    off = 0

    def u32():
        nonlocal off
        v = struct.unpack_from("<I", data, off)[0]
        off += 4
        return v

    def f32s(n):
        nonlocal off
        v = struct.unpack_from(f"<{n}f", data, off)
        off += 4 * n
        return v

    version = u32()
    headers = []
    if version == 1:
        n = u32()
        for _ in range(n):
            f32s(6)
            headers.append((u32(), u32(), u32()))
    if version == 0:
        headers.append(None)
    for t in range(topo):
        if version == 0:
            f32s(6)
            ic = u32()
            off += ic * 4
            vc = u32()
            off += vc * 8
            tf = u32()
            off += (tf // 3) * 24
        else:
            ic, vc, fc = headers[t]
            off += ic * 4 + vc * 8 + vc * fc * 24
    if version == 0:
        f32s(6)
        ic = u32()
        indices = struct.unpack_from(f"<{ic}I", data, off)
        off += ic * 4
        vc = u32()
    else:
        ic, vc, fc = headers[topo]
        indices = struct.unpack_from(f"<{ic}I", data, off)
        off += ic * 4
    f32s(vc * 2)
    if version == 0:
        tf = u32()
        tv = tf // 3
        fc = tv // vc
    else:
        tv = vc * fc
    pos = np.array(f32s(tv * 3)).reshape(tv, 3)
    base = frame * vc
    return pos[base:base + vc].copy(), np.array(indices).reshape(-1, 3).copy()


def parse_tetcage(path):
    data = open(path, "rb").read()
    off = 8
    struct.unpack_from("<I", data, off); off += 4

    def u32():
        nonlocal off
        v = struct.unpack_from("<I", data, off)[0]
        off += 4
        return v

    def i32():
        nonlocal off
        v = struct.unpack_from("<i", data, off)[0]
        off += 4
        return v

    def f32():
        nonlocal off
        v = struct.unpack_from("<f", data, off)[0]
        off += 4
        return v

    f32(), f32(), f32()
    h = f32()
    u32(), u32(), u32()
    ncv, nt, npv, npt = u32(), u32(), u32(), u32()
    u32(), u32()
    cagev_off = off
    off += ncv * 12
    tets = []
    for _ in range(nt):
        off += 16
        nb = [i32() for _ in range(4)]
        vs, vc, ts, tc = u32(), u32(), u32(), u32()
        tets.append((nb, vs, ts, tc))
    pv = np.frombuffer(data, dtype="<f4", count=npv * 8, offset=off).reshape(npv, 8)
    off += npv * 8 * 4
    tri = np.frombuffer(data, dtype="<u4", count=npt * 3, offset=off).reshape(npt, 3)
    cagev = np.frombuffer(data, dtype="<f4", count=ncv * 3, offset=cagev_off).reshape(ncv, 3)
    return h, tets, pv[:, 0:3].copy(), tri, cagev


def weld(positions, tol):
    n = len(positions)
    grid = np.floor(positions / tol).astype(np.int64)
    buckets = defaultdict(list)
    for i in range(n):
        buckets[tuple(grid[i])].append(i)
    ids = np.full(n, -1, dtype=np.int64)
    anchors = []
    for i in range(n):
        if ids[i] >= 0:
            continue
        aid = len(anchors)
        anchors.append(i)
        gi = grid[i]
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for j in buckets.get((gi[0] + dx, gi[1] + dy, gi[2] + dz), ()):
                        if ids[j] < 0:
                            dp = positions[j] - positions[i]
                            if dp @ dp <= tol * tol:
                                ids[j] = aid
    return ids


def seg_point_dist(p, a, b):
    ab = b - a
    denom = ab @ ab
    if denom < 1e-30:
        return float(np.linalg.norm(p - a))
    t = np.clip((p - a) @ ab / denom, 0.0, 1.0)
    return float(np.linalg.norm(p - (a + t * ab)))


def main(cage_path, vat_path, topo):
    srcP, srcT = load_vat(vat_path, topo)
    h, tets, pos, tri, cagev = parse_tetcage(cage_path)
    print(f"file: {cage_path}")
    print(f"source: {len(srcP)} verts, {len(srcT)} tris; cage h={h:.6g}, {len(tets)} tets, "
          f"{len(pos)} piece verts, {len(tri)} piece tris")

    # source boundary edges (by index)
    sec = defaultdict(int)
    for (a, b, c) in srcT:
        for u, v in ((a, b), (b, c), (c, a)):
            sec[(min(u, v), max(u, v))] += 1
    src_b = [(srcP[a], srcP[b]) for (a, b), c in sec.items() if c == 1]
    src_nm = sum(1 for c in sec.values() if c > 2)
    print(f"source: {len(src_b)} boundary edges, {src_nm} non-manifold")

    # all piece triangles with owning tet
    tris_flat = []
    for t, (nb, vs, ts, tc) in enumerate(tets):
        for (a, b, c) in tri[ts:ts + tc]:
            tris_flat.append((t, a + vs, b + vs, c + vs))

    for wtol in (h * 1e-6, h * 1e-5, h * 1e-4, h * 1e-3):
        ids = weld(pos, wtol)
        use = defaultdict(list)
        for (t, a, b, c) in tris_flat:
            w = [int(ids[a]), int(ids[b]), int(ids[c])]
            for u, v in ((w[0], w[1]), (w[1], w[2]), (w[2], w[0])):
                if u == v:
                    continue
                use[(min(u, v), max(u, v))].append((t, u, v))
        boundary = {e: v for e, v in use.items() if len(v) == 1}
        nonman = sum(1 for v in use.values() if len(v) > 2)
        # classify boundary rims
        on_src_b = 0
        interior = []
        for e, v in boundary.items():
            (t, u, vv) = v[0]
            # welded anchor position: any copy works (they are within wtol)
            pa, pb = pos[u], pos[vv]
            d = min((min(seg_point_dist(pa, x, y), seg_point_dist(pb, x, y))
                     for (x, y) in src_b), default=1e30)
            if d <= h * 1e-2:
                on_src_b += 1
            else:
                interior.append((t, pa, pb))
        print(f"weld {wtol:.2e} ({wtol/h:.0e} h): boundary={len(boundary)} "
              f"[src-boundary={on_src_b}, interior={len(interior)}], nonmanifold={nonman}")
        if wtol == h * 1e-5 and interior:
            lens = sorted(np.linalg.norm(pb - pa) for (_, pa, pb) in interior)
            print(f"  interior rim lengths: max={lens[-1]:.2e} "
                  f"median={lens[len(lens)//2]:.2e}")
            for (t, pa, pb) in interior[:8]:
                print(f"    tet={t} {np.array2string(pa, precision=4)} -> "
                      f"{np.array2string(pb, precision=4)} len={np.linalg.norm(pb-pa):.2e}")


if __name__ == "__main__":
    args = sys.argv[1:]
    cage = args[0] if args else "assets/models/ginkgo/ginkgo0.tetcage"
    vat = args[1] if len(args) > 1 else "assets/models/ginkgo/ginkgo0.vat"
    topo = int(args[2]) if len(args) > 2 else 0
    main(cage, vat, topo)
