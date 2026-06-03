#!/usr/bin/env python3
"""Kernel 2 — Fused safety analysis.

The safety monitor node runs four independent passes over the depth image:
obstacle, stair, drop, narrow-passage. This module provides three
implementations of the same four metrics so the project can run an honest
three-way comparison:

  1. ``safety_original_numpy``  — the node's exact logic (percentile / median /
     Python for-loop). This is the unmodified baseline.
  2. ``safety_optimized_numpy`` — vectorised, no Python loops, using the
     reduction-friendly definitions below. This is the fair CPU baseline.
  3. ``FusedSafetyCuPy``        — one fused CuPy RawKernel that traverses the
     full frame once, accumulating every per-column / per-region statistic with
     atomics, then a trivial finalise over 640 columns.

INTELLECTUAL-HONESTY NOTE (read before quoting any speedup):
The optimized + CuPy paths deliberately use **mean / min** reductions where the
original node uses **median / 5th-percentile**. Median and exact percentile
need a sort, which does not fuse into a single streaming pass. This is a
documented *semantic* change, not free speedup:
  * obstacle: original = 5th percentile of ROI; optimized/CuPy = min of ROI.
  * drop: original = median(far) - median(near); optimized/CuPy = mean diff.
  * stair + passage: identical definitions across all three.
When reporting, separate "original -> optimized" (vectorisation + semantic
change) from "optimized -> CuPy" (pure parallelisation, exact parity to tol).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# Robot physical parameters — mirror safety_monitor_node.py
ROBOT_WIDTH_M = 0.35
MIN_PASSAGE_WIDTH_M = 0.55
MAX_STEP_HEIGHT_M = 0.08
STOP_DISTANCE_M = 0.4
SLOWDOWN_DISTANCE_M = 1.0


@dataclass
class SafetyResult:
    min_obstacle_dist: float  # metres (inf if none)
    stair_max_gradient: float  # metres
    drop_magnitude: float  # metres (far - near)
    passage_width_m: float  # metres (inf if no close obstacle in mid row)

    def close(self, other: "SafetyResult", atol: float = 1e-3) -> bool:
        def eq(a, b):
            if np.isinf(a) and np.isinf(b):
                return True
            return abs(a - b) <= atol
        return (
            eq(self.min_obstacle_dist, other.min_obstacle_dist)
            and eq(self.stair_max_gradient, other.stair_max_gradient)
            and eq(self.drop_magnitude, other.drop_magnitude)
            and eq(self.passage_width_m, other.passage_width_m)
        )


def _longest_clear_run(clear_mask: np.ndarray) -> int:
    """Longest run of True in a 1-D boolean array (vectorised)."""
    if not clear_mask.any():
        return 0
    # Boundaries where value changes; compute run lengths of True segments.
    idx = np.flatnonzero(np.diff(np.concatenate(([0], clear_mask.view(np.int8), [0]))))
    runs = idx[1::2] - idx[0::2]
    return int(runs.max()) if runs.size else 0


# ----------------------------------------------------------------------------
# 1. Original NumPy — node's exact logic (percentile / median / for-loop)
# ----------------------------------------------------------------------------


def safety_original_numpy(
    depth_m: np.ndarray, fx: float, max_depth: float = 5.0, floor_band: float = 0.15
) -> SafetyResult:
    h, w = depth_m.shape

    # 1. obstacle: 5th percentile of center ROI
    roi = depth_m[h // 3 : 2 * h // 3, w // 3 : 2 * w // 3]
    valid = roi[(roi > 0) & (roi < max_depth)]
    min_dist = float(np.percentile(valid, 5)) if valid.size > 0 else float("inf")

    # 2. stair: max |diff(column means)| in floor band
    floor_row_start = int(h * (1.0 - floor_band))
    fb = depth_m[floor_row_start:, :]
    valid_floor = np.where((fb > 0) & (fb < max_depth), fb, np.nan)
    stair = 0.0
    if not np.all(np.isnan(valid_floor)):
        col_means = np.nanmean(valid_floor, axis=0)
        grad = np.abs(np.diff(col_means))
        grad = grad[~np.isnan(grad)]
        stair = float(np.max(grad)) if grad.size > 0 else 0.0

    # 3. drop: median(far) - median(near)
    near = depth_m[h - 20 : h, w // 3 : 2 * w // 3]
    far = depth_m[h // 2 : h // 2 + 20, w // 3 : 2 * w // 3]
    nv = near[(near > 0) & (near < max_depth)]
    fv = far[(far > 0) & (far < max_depth)]
    drop = (float(np.median(fv)) - float(np.median(nv))) if nv.size > 10 and fv.size > 10 else 0.0

    # 4. passage: longest clear run in mid row (Python for-loop in node)
    mid = depth_m[h // 2, :]
    close_mask = (mid > 0) & (mid < SLOWDOWN_DISTANCE_M)
    gap = 0
    cur = 0
    for v in close_mask:
        if not v:
            cur += 1
            gap = max(gap, cur)
        else:
            cur = 0
    passage = gap * SLOWDOWN_DISTANCE_M / fx if (fx and fx > 0) else float("inf")
    if not np.any(close_mask):
        passage = float("inf")

    return SafetyResult(min_dist, stair, drop, float(passage))


# ----------------------------------------------------------------------------
# 2. Optimized NumPy — reduction-friendly definitions (mean / min), no loops.
#    This is the EXACT reference the CuPy kernel must match to tolerance.
# ----------------------------------------------------------------------------


def safety_optimized_numpy(
    depth_m: np.ndarray, fx: float, max_depth: float = 5.0, floor_band: float = 0.15
) -> SafetyResult:
    h, w = depth_m.shape

    roi = depth_m[h // 3 : 2 * h // 3, w // 3 : 2 * w // 3]
    vmask = (roi > 0) & (roi < max_depth)
    min_dist = float(roi[vmask].min()) if vmask.any() else float("inf")

    floor_row_start = int(h * (1.0 - floor_band))
    fb = depth_m[floor_row_start:, :]
    fbv = (fb > 0) & (fb < max_depth)
    col_cnt = fbv.sum(axis=0)
    col_sum = np.where(fbv, fb, 0.0).sum(axis=0)
    col_means = np.divide(col_sum, col_cnt, out=np.full(w, np.nan), where=col_cnt > 0)
    grad = np.abs(np.diff(col_means))
    grad = grad[~np.isnan(grad)]
    stair = float(grad.max()) if grad.size > 0 else 0.0

    near = depth_m[h - 20 : h, w // 3 : 2 * w // 3]
    far = depth_m[h // 2 : h // 2 + 20, w // 3 : 2 * w // 3]
    nm = (near > 0) & (near < max_depth)
    fm = (far > 0) & (far < max_depth)
    drop = (
        float(far[fm].mean()) - float(near[nm].mean())
        if nm.sum() > 10 and fm.sum() > 10
        else 0.0
    )

    mid = depth_m[h // 2, :]
    close_mask = (mid > 0) & (mid < SLOWDOWN_DISTANCE_M)
    clear = ~close_mask
    gap = _longest_clear_run(clear)
    passage = gap * SLOWDOWN_DISTANCE_M / fx if (fx and fx > 0) else float("inf")
    if not np.any(close_mask):
        passage = float("inf")

    return SafetyResult(min_dist, stair, drop, float(passage))


# ----------------------------------------------------------------------------
# 3. Fused CuPy RawKernel — single full-frame pass with atomics
# ----------------------------------------------------------------------------

_FUSED_SRC = r"""
extern "C" __global__
void fused_safety(
        const unsigned short* __restrict__ depth,  // HxW mm
        float* __restrict__ col_sum,    // [w] floor-band per-column sum (metres)
        int*   __restrict__ col_cnt,    // [w] floor-band per-column valid count
        unsigned char* __restrict__ mid_close, // [w] mid-row close flag
        float* __restrict__ near_sum,   // [1]
        int*   __restrict__ near_cnt,   // [1]
        float* __restrict__ far_sum,    // [1]
        int*   __restrict__ far_cnt,    // [1]
        int*   __restrict__ obst_min_mm,// [1] atomicMin, init large
        const int h, const int w,
        const float max_depth_m, const int floor_row_start,
        const float slowdown_m)
{
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    const int n = h * w;
    if (idx >= n) return;

    const int v = idx / w;
    const int u = idx - v * w;
    const int dmm = (int)depth[idx];
    const float dm = dmm * 0.001f;
    const int valid = (dm > 0.0f) && (dm < max_depth_m);

    const int c0 = w / 3, c1 = 2 * w / 3;
    const int r0 = h / 3, r1 = 2 * h / 3;

    // obstacle: min over center ROI
    if (valid && v >= r0 && v < r1 && u >= c0 && u < c1) {
        atomicMin(obst_min_mm, dmm);
    }
    // stair: floor-band per-column sum/count
    if (valid && v >= floor_row_start) {
        atomicAdd(&col_sum[u], dm);
        atomicAdd(&col_cnt[u], 1);
    }
    // drop: near = last 20 rows, far = 20 rows from h/2, both center cols
    if (valid && u >= c0 && u < c1) {
        if (v >= h - 20 && v < h) { atomicAdd(near_sum, dm); atomicAdd(near_cnt, 1); }
        if (v >= h / 2 && v < h / 2 + 20) { atomicAdd(far_sum, dm); atomicAdd(far_cnt, 1); }
    }
    // passage: mid row close flag (single row, one writer per column)
    if (v == h / 2) {
        mid_close[u] = (dm > 0.0f && dm < slowdown_m) ? 1 : 0;
    }
}
"""


class FusedSafetyCuPy:
    def __init__(self, threads_per_block: int = 256):
        import cupy as cp

        self._cp = cp
        self._kernel = cp.RawKernel(_FUSED_SRC, "fused_safety")
        self._tpb = int(threads_per_block)

    def __call__(
        self, depth_m, fx: float, max_depth: float = 5.0, floor_band: float = 0.15
    ) -> SafetyResult:
        cp = self._cp
        dm = cp.asarray(depth_m, dtype=cp.float32)
        depth_mm = cp.asarray(cp.rint(dm * 1000.0), dtype=cp.uint16)
        h, w = dm.shape
        n = h * w
        floor_row_start = int(h * (1.0 - floor_band))

        col_sum = cp.zeros(w, dtype=cp.float32)
        col_cnt = cp.zeros(w, dtype=cp.int32)
        mid_close = cp.zeros(w, dtype=cp.uint8)
        near_sum = cp.zeros(1, dtype=cp.float32)
        near_cnt = cp.zeros(1, dtype=cp.int32)
        far_sum = cp.zeros(1, dtype=cp.float32)
        far_cnt = cp.zeros(1, dtype=cp.int32)
        obst_min_mm = cp.full(1, 1 << 30, dtype=cp.int32)

        blocks = (n + self._tpb - 1) // self._tpb
        self._kernel(
            (blocks,),
            (self._tpb,),
            (
                depth_mm.ravel(), col_sum, col_cnt, mid_close,
                near_sum, near_cnt, far_sum, far_cnt, obst_min_mm,
                np.int32(h), np.int32(w), np.float32(max_depth),
                np.int32(floor_row_start), np.float32(SLOWDOWN_DISTANCE_M),
            ),
        )
        # ---- trivial finalise over w columns (host) ----
        col_sum_h = cp.asnumpy(col_sum)
        col_cnt_h = cp.asnumpy(col_cnt)
        mid_h = cp.asnumpy(mid_close).astype(bool)
        col_means = np.divide(
            col_sum_h, col_cnt_h, out=np.full(w, np.nan), where=col_cnt_h > 0
        )
        grad = np.abs(np.diff(col_means))
        grad = grad[~np.isnan(grad)]
        stair = float(grad.max()) if grad.size > 0 else 0.0

        nc = int(near_cnt.item())
        fc = int(far_cnt.item())
        drop = (
            float(far_sum.item()) / fc - float(near_sum.item()) / nc
            if nc > 10 and fc > 10
            else 0.0
        )

        omm = int(obst_min_mm.item())
        min_dist = omm / 1000.0 if omm < (1 << 30) else float("inf")

        clear = ~mid_h
        gap = _longest_clear_run(clear)
        passage = gap * SLOWDOWN_DISTANCE_M / fx if (fx and fx > 0) else float("inf")
        if not mid_h.any():
            passage = float("inf")

        return SafetyResult(min_dist, stair, drop, float(passage))
