#!/usr/bin/env python3
"""Trace source-edge sub-segment chains through the piece soup.

For a chosen source edge (or the worst rims automatically), collect every
piece edge that lies on it, order them along the edge, and report per-tet
coverage: which tets contribute sub-segments, endpoint mismatches between
consecutive sub-segments, and gaps (uncovered intervals) along the edge.

Also measures, for every welded-boundary rim, the nearest coincident rim
(endpoint Hausdorff, both orientations) among ALL rims — the crack-width
distribution of rim-vs-rim mismatches.
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
    off = 0

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

    assert data[:8] == b"TETCAGE1"
    off = 8
    assert u32() == 1
    f32(), f32(), f32()
    h = f32()
    u32(), u32(), u32()
    ncv, nt, npv, npt = u32(), u32(), u32(), u32()
    u32(), u32()
    off += ncv * 12
    cagev_off, tet_off = off, off + nt * (16 + 16 + 16)
    tets = []
    for _ in range(nt):
        off += 16  # cage vert ids
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


def point_seg_dist(p, a, b):
    ab = b - a
    denom = ab @ ab
    if denom < 1e-30:
        return float(np.linalg.norm(p - a))
    t = np.clip((p - a) @ ab / denom, 0.0, 1.0)
    return float(np.linalg.norm(p - (a + t * ab)))


def main(cage_path, vat_path, topo):
    srcP, srcT = load_vat(vat_path, topo)
    h, tets, pos, tri, cagev = parse_tetcage(cage_path)
    print(f"h={h:.6g} tets={len(tets)} src tris={len(srcT)}")

    # per-piece edges
    tet_edges = []
    for (nb, vs, ts, tc) in tets:
        seen = set()
        el = []
        for (a, b, c) in tri[ts:ts + tc]:
            a, b, c = a + vs, b + vs, c + vs
            for u, v in ((a, b), (b, c), (c, a)):
                k = (min(u, v), max(u, v))
                if k not in seen:
                    seen.add(k)
                    el.append((u, v))
        tet_edges.append(el)

    ids = weld(pos, h * 1e-5)
    wcount = defaultdict(list)
    for t, el in enumerate(tet_edges):
        for (a, b) in el:
            wa, wb = int(ids[a]), int(ids[b])
            if wa == wb:
                continue
            wcount[(min(wa, wb), max(wa, wb))].append((t, a, b))
    rims = [v[0] for v in wcount.values() if len(v) == 1]
    print(f"welded boundary rims: {len(rims)}")

    # ---- rim-vs-rim nearest coincident partner (endpoint Hausdorff, both
    # orientations), vectorized over all rims
    R = np.array([[pos[a], pos[b]] for (t, a, b) in rims])  # (n,2,3)
    n = len(R)
    A0, B0 = R[:, 0], R[:, 1]
    # pairwise endpoint distance matrices in blocks
    best = np.full(n, np.inf)
    blk = 256
    for s in range(0, n, blk):
        e = min(n, s + blk)
        a0, b0 = A0[s:e], B0[s:e]                       # (m,3)
        # dist of each rim-i endpoint to rim-j endpoints
        d00 = np.linalg.norm(a0[:, None, :] - A0[None, :, :], axis=2)
        d01 = np.linalg.norm(a0[:, None, :] - B0[None, :, :], axis=2)
        d10 = np.linalg.norm(b0[:, None, :] - A0[None, :, :], axis=2)
        d11 = np.linalg.norm(b0[:, None, :] - B0[None, :, :], axis=2)
        m1 = np.maximum(d00, d11)   # same orientation
        m2 = np.maximum(d01, d10)   # swapped
        mm = np.minimum(m1, m2)
        for i_ in range(e - s):
            mm[i_, s + i_] = np.inf  # self
            # also exclude reversed-duplicate of the same edge if present
        best[s:e] = mm.min(axis=1)
    finite = best[np.isfinite(best)]
    print("rim -> nearest coincident rim endpoint-Hausdorff:")
    for thr in (h * 1e-6, h * 1e-5, h * 1e-4, h * 1e-3, h * 1e-2, h * 0.1, h):
        print(f"  within {thr:.2e} ({thr/h:.0e} h): {int((finite <= thr).sum())}/{n}")
    print(f"  max={finite.max():.3e}, median={np.median(finite):.3e}")

    # ---- trace the source edge owning the worst rim chains
    # source edges
    src_edges = set()
    for (a, b, c) in srcT:
        for u, v in ((a, b), (b, c), (c, a)):
            src_edges.add((min(u, v), max(u, v)))
    src_edges = list(src_edges)

    def trace(pa, pb, label):
        # find source edge containing this point
        bestse, bd = None, 1e30
        for (u, v) in src_edges:
            d = min(point_seg_dist(pa, srcP[u], srcP[v]), point_seg_dist(pb, srcP[u], srcP[v]))
            if d < bd:
                bd, bestse = d, (u, v)
        (u, v) = bestse
        a, b = srcP[u], srcP[v]
        ab = b - a
        L = np.linalg.norm(ab)
        print(f"\n== trace {label}: rim near src edge {u}-{v} len={L:.4f} "
              f"a={np.array2string(a, precision=4)} b={np.array2string(b, precision=4)} (dist {bd:.1e})")
        # collect ALL piece edges lying on this source edge (both endpoints on it)
        subs = []
        for t, el in enumerate(tet_edges):
            for (x, y) in el:
                px, py = pos[x], pos[y]
                if (point_seg_dist(px, a, b) < h * 1e-3 and point_seg_dist(py, a, b) < h * 1e-3):
                    s0 = float((px - a) @ ab / (L * L))
                    s1 = float((py - a) @ ab / (L * L))
                    is_rim = any(rr[1] == x and rr[2] == y for rr in rims)
                    subs.append((min(s0, s1), max(s0, s1), t, is_rim,
                                 min(point_seg_dist(px, a, b), 0), s0, s1))
        subs.sort()
        print(f"  {len(subs)} piece sub-segments lie on this source edge:")
        prev_end = None
        for (s0, s1, t, is_rim, _, q0, q1) in subs:
            gap = "" if prev_end is None else f" gap={s0 - prev_end:+.2e}"
            print(f"   tet={t:5d} [{s0:.4f},{s1:.4f}] len={s1-s0:.2e} rim={int(is_rim)}{gap}")
            prev_end = max(prev_end or s1, s1)
        # union coverage gaps
        cov = []
        cur = None
        for (s0, s1, *_rest) in subs:
            if cur is None:
                cur = [s0, s1]
            elif s0 <= cur[1] + 1e-9:
                cur[1] = max(cur[1], s1)
            else:
                cov.append(tuple(cur))
                cur = [s0, s1]
        if cur:
            cov.append(tuple(cur))
        gaps = [(cov[i][1], cov[i + 1][0]) for i in range(len(cov) - 1)]
        print(f"  coverage: {len(cov)} intervals, gaps: {[(f'{x:.4f}',f'{y:.4f}',f'{y-x:.2e}') for x,y in gaps]}")

    # trace the two chains seen earlier: pick rims by nearest-chain heuristic —
    # use the worst rim from before (tet 46/47 region) and the biggest rim
    rim_arr = [(t, a, b) for (t, a, b) in rims]
    by_len = sorted(rim_arr, key=lambda r: -np.linalg.norm(pos[r[2]] - pos[r[1]]))
    trace(pos[by_len[0][1]], pos[by_len[0][2]], "longest rim")
    # a rim from the tet 46-53 cluster: find rim owned by tet 46
    for (t, a, b) in rim_arr:
        if t in (46, 47):
            trace(pos[a], pos[b], f"tet{t} rim")
            break


if __name__ == "__main__":
    args = sys.argv[1:]
    cage = args[0] if args else "assets/models/ginkgo/ginkgo0.tetcage"
    vat = args[1] if len(args) > 1 else "assets/models/ginkgo/ginkgo0.vat"
    topo = int(args[2]) if len(args) > 2 else 0
    main(cage, vat, topo)
