#!/usr/bin/env python3
"""Parity tests: CuPy kernels vs NumPy references.

Skips automatically if CuPy / a usable GPU is unavailable, so the suite stays
green in CI on machines without a CUDA device.
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kernels.depth_to_pointcloud import DepthToPointCloudCuPy, backproject_numpy
from kernels.fused_safety import (
    FusedSafetyCuPy,
    safety_optimized_numpy,
    safety_original_numpy,
)

cp = pytest.importorskip("cupy")


def _gpu_ok() -> bool:
    try:
        cp.arange(1)  # prebuilt kernel; may fail on arch mismatch
        return True
    except Exception:
        # RawKernel JIT can still work even if prebuilt kernels do not.
        try:
            k = cp.RawKernel(
                r'extern "C" __global__ void t(float* y){y[0]=1.0f;}', "t"
            )
            y = cp.empty(1, dtype=cp.float32)
            k((1,), (1,), (y,))
            cp.cuda.runtime.deviceSynchronize()
            return True
        except Exception:
            return False


pytestmark = pytest.mark.skipif(not _gpu_ok(), reason="no usable CUDA GPU")


def _synthetic_depth(seed=0, h=480, w=640):
    rng = np.random.default_rng(seed)
    # mix of valid depths (300-6000mm), zeros (invalid), and a structured ramp
    depth = rng.integers(0, 6000, size=(h, w)).astype(np.uint16)
    depth[rng.random((h, w)) < 0.1] = 0  # 10% invalid
    depth[-60:, :] += np.arange(w, dtype=np.uint16)[None, :]  # floor ramp for stair
    return depth


# ---- Kernel 1 ----

def test_pointcloud_parity():
    depth = _synthetic_depth(1)
    fx, fy, cx, cy = 600.0, 600.0, 320.0, 240.0
    pts_np, mask_np = backproject_numpy(depth, fx, fy, cx, cy, max_depth_m=8.0)
    k = DepthToPointCloudCuPy()
    pts_cp, mask_cp = k(depth, fx, fy, cx, cy, max_depth_m=8.0)
    pts_cp = cp.asnumpy(pts_cp)
    mask_cp = cp.asnumpy(mask_cp)
    assert np.array_equal(mask_np, mask_cp)
    assert np.allclose(pts_np, pts_cp, atol=1e-4), np.abs(pts_np - pts_cp).max()


def test_pointcloud_all_invalid():
    depth = np.zeros((480, 640), dtype=np.uint16)
    k = DepthToPointCloudCuPy()
    pts_cp, mask_cp = k(depth, 600, 600, 320, 240, 8.0)
    assert int(cp.asnumpy(mask_cp).sum()) == 0
    assert float(cp.asnumpy(pts_cp).sum()) == 0.0


# ---- Kernel 2 ----

def test_fused_safety_parity_optimized():
    """CuPy fused kernel must match optimized NumPy to tolerance."""
    depth_mm = _synthetic_depth(2)
    depth_m = depth_mm.astype(np.float32) / 1000.0
    fx = 600.0
    ref = safety_optimized_numpy(depth_m, fx, max_depth=5.0)
    got = FusedSafetyCuPy()(depth_m, fx, max_depth=5.0)
    assert got.close(ref, atol=2e-3), (ref, got)


def test_fused_safety_documented_delta_vs_original():
    """Original (median/percentile) and optimized (mean/min) differ — this is a
    documented semantic change, asserted here so it is never mistaken for a bug."""
    depth_mm = _synthetic_depth(3)
    depth_m = depth_mm.astype(np.float32) / 1000.0
    orig = safety_original_numpy(depth_m, 600.0)
    opt = safety_optimized_numpy(depth_m, 600.0)
    # stair and passage definitions are identical -> should match closely
    assert abs(orig.stair_max_gradient - opt.stair_max_gradient) < 1e-3
    assert abs(orig.passage_width_m - opt.passage_width_m) < 1e-3
    # obstacle (pctile vs min) and drop (median vs mean) are expected to differ
    assert orig.min_obstacle_dist >= opt.min_obstacle_dist - 1e-6


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
