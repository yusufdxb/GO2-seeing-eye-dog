#!/usr/bin/env python3
"""Kernel 1 — Depth-to-Pointcloud back-projection.

Converts a uint16 depth image (millimetres) into a dense organised point
cloud (X, Y, Z in metres, camera frame) using the pinhole model:

    Z = depth_mm / 1000
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy

A pixel is *valid* when 0 < Z <= max_depth_m.

Two implementations with identical semantics:
  * ``backproject_numpy``  — vectorised NumPy reference (ground truth)
  * ``DepthToPointCloudCuPy`` — CuPy RawKernel, one CUDA thread per pixel

This is the "embarrassingly parallel, no shared memory" kernel: 640x480 =
307200 independent threads, each doing 3 multiplies and a compare. It is the
honest place GPU acceleration is expected to help (large element count, no
reduction), in contrast to the per-detection depth sampling in the perception
node which operates on a handful of ROIs and is left on the CPU on purpose.
"""

from __future__ import annotations

import numpy as np

# ----------------------------------------------------------------------------
# NumPy reference (ground truth)
# ----------------------------------------------------------------------------


def backproject_numpy(
    depth_mm: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    max_depth_m: float = 8.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Vectorised back-projection. Returns (points HxWx3 float32, mask HxW uint8)."""
    h, w = depth_mm.shape
    z = depth_mm.astype(np.float32) / 1000.0
    us = np.arange(w, dtype=np.float32)[None, :]
    vs = np.arange(h, dtype=np.float32)[:, None]
    x = (us - cx) * z / fx
    y = (vs - cy) * z / fy
    points = np.stack([x, y, z], axis=-1).astype(np.float32)
    mask = ((z > 0.0) & (z <= max_depth_m)).astype(np.uint8)
    # Invalid points zeroed so the two backends agree bit-for-bit on them.
    points[mask == 0] = 0.0
    return points, mask


# ----------------------------------------------------------------------------
# CuPy RawKernel
# ----------------------------------------------------------------------------

_KERNEL_SRC = r"""
extern "C" __global__
void depth_to_pointcloud(
        const unsigned short* __restrict__ depth,   // HxW, millimetres
        float* __restrict__ points,                 // HxWx3, metres
        unsigned char* __restrict__ mask,           // HxW
        const int h, const int w,
        const float fx, const float fy,
        const float cx, const float cy,
        const float max_depth_m)
{
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    const int n = h * w;
    if (idx >= n) return;

    const int v = idx / w;
    const int u = idx - v * w;

    const float z = (float)depth[idx] * 0.001f;   // mm -> m
    const int valid = (z > 0.0f) && (z <= max_depth_m);

    const int o = idx * 3;
    if (valid) {
        points[o]     = ((float)u - cx) * z / fx;
        points[o + 1] = ((float)v - cy) * z / fy;
        points[o + 2] = z;
        mask[idx] = 1;
    } else {
        points[o] = 0.0f;
        points[o + 1] = 0.0f;
        points[o + 2] = 0.0f;
        mask[idx] = 0;
    }
}
"""


class DepthToPointCloudCuPy:
    """CuPy RawKernel wrapper. One thread per pixel."""

    def __init__(self, threads_per_block: int = 256):
        import cupy as cp  # lazy: keeps module importable without a GPU

        self._cp = cp
        self._kernel = cp.RawKernel(_KERNEL_SRC, "depth_to_pointcloud")
        self._tpb = int(threads_per_block)

    def __call__(
        self,
        depth_mm,  # cupy uint16 array HxW (or numpy, auto-uploaded)
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        max_depth_m: float = 8.0,
    ):
        cp = self._cp
        d = cp.asarray(depth_mm, dtype=cp.uint16)
        h, w = d.shape
        n = h * w
        points = cp.empty((h, w, 3), dtype=cp.float32)
        mask = cp.empty((h, w), dtype=cp.uint8)
        blocks = (n + self._tpb - 1) // self._tpb
        self._kernel(
            (blocks,),
            (self._tpb,),
            (
                d.ravel(),
                points.ravel(),
                mask.ravel(),
                np.int32(h),
                np.int32(w),
                np.float32(fx),
                np.float32(fy),
                np.float32(cx),
                np.float32(cy),
                np.float32(max_depth_m),
            ),
        )
        return points, mask
