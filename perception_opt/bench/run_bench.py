#!/usr/bin/env python3
"""Automated benchmark harness — RTX 5070 (ablation config 8).

Measures, with warmup + synchronisation + percentile statistics:
  * Kernel 1 (depth->pointcloud): NumPy vs CuPy
  * Kernel 2 (fused safety): original NumPy vs optimized NumPy vs CuPy
  * Kernel launch overhead via CUDA events

Honesty rules enforced (see vault status.md "Anti-Misleading Rules"):
  * 100-iteration warmup discarded
  * cp.cuda.runtime.deviceSynchronize() before every timer stop
  * wall time INCLUDES host<->device transfer (the real cost in the pipeline)
  * reports mean/std/p50/p95/p99, never a single best
  * platform tag recorded; this is RTX 5070 only, NOT Jetson.

Jetson configs (1-7) require physical hardware and are intentionally not run
here; they are marked BLOCKED-HW in the vault.
"""
from __future__ import annotations

import argparse
import json
import platform
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from kernels.depth_to_pointcloud import DepthToPointCloudCuPy, backproject_numpy
from kernels.fused_safety import (
    FusedSafetyCuPy,
    safety_optimized_numpy,
    safety_original_numpy,
)


def synthetic_depth(seed=0, h=480, w=640):
    rng = np.random.default_rng(seed)
    depth = rng.integers(0, 6000, size=(h, w)).astype(np.uint16)
    depth[rng.random((h, w)) < 0.1] = 0
    depth[-60:, :] += np.arange(w, dtype=np.uint16)[None, :]
    return depth


def stats(samples_ms: list[float]) -> dict:
    a = np.asarray(samples_ms, dtype=np.float64)
    return {
        "n": int(a.size),
        "mean_ms": float(a.mean()),
        "std_ms": float(a.std()),
        "p50_ms": float(np.percentile(a, 50)),
        "p95_ms": float(np.percentile(a, 95)),
        "p99_ms": float(np.percentile(a, 99)),
        "min_ms": float(a.min()),
        "max_ms": float(a.max()),
    }


def time_cpu(fn, iters, warmup):
    for _ in range(warmup):
        fn()
    samples = []
    for _ in range(iters):
        t0 = time.perf_counter_ns()
        fn()
        samples.append((time.perf_counter_ns() - t0) / 1e6)
    return stats(samples)


def time_gpu_wall(fn, iters, warmup):
    """Wall-clock incl. transfer + sync — the honest end-to-end GPU cost."""
    import cupy as cp

    for _ in range(warmup):
        fn()
    cp.cuda.runtime.deviceSynchronize()
    samples = []
    for _ in range(iters):
        t0 = time.perf_counter_ns()
        fn()
        cp.cuda.runtime.deviceSynchronize()
        samples.append((time.perf_counter_ns() - t0) / 1e6)
    return stats(samples)


def kernel_launch_overhead(iters=2000):
    """Pure launch overhead of a no-op kernel via CUDA events (microseconds)."""
    import cupy as cp

    noop = cp.RawKernel(r'extern "C" __global__ void noop(){}', "noop")
    start = cp.cuda.Event()
    end = cp.cuda.Event()
    for _ in range(100):
        noop((1,), (1,), ())
    cp.cuda.runtime.deviceSynchronize()
    samples_us = []
    for _ in range(iters):
        start.record()
        noop((1,), (1,), ())
        end.record()
        end.synchronize()
        samples_us.append(cp.cuda.get_elapsed_time(start, end) * 1000.0)  # ms->us
    a = np.asarray(samples_us)
    return {"mean_us": float(a.mean()), "p50_us": float(np.percentile(a, 50)),
            "p99_us": float(np.percentile(a, 99))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=1000)
    ap.add_argument("--warmup", type=int, default=100)
    ap.add_argument("--out", default=str(ROOT / "results" / "bench_rtx5070.json"))
    args = ap.parse_args()

    import cupy as cp

    dev = cp.cuda.runtime.getDeviceProperties(0)["name"].decode()
    cc = cp.cuda.Device().compute_capability

    depth = synthetic_depth(0)
    depth_m = depth.astype(np.float32) / 1000.0
    fx, fy, cx, cy = 600.0, 600.0, 320.0, 240.0

    results = {
        "platform": {
            "device": dev, "compute_capability": cc, "host": platform.node(),
            "note": "RTX 5070 only (ablation config 8). Jetson configs BLOCKED-HW.",
        },
        "config": {"iters": args.iters, "warmup": args.warmup, "resolution": "640x480"},
    }

    # ---- Kernel 1 ----
    pc = DepthToPointCloudCuPy()
    results["kernel1_pointcloud"] = {
        "numpy": time_cpu(lambda: backproject_numpy(depth, fx, fy, cx, cy, 8.0),
                          args.iters, args.warmup),
        "cupy_wall": time_gpu_wall(lambda: pc(depth, fx, fy, cx, cy, 8.0),
                                   args.iters, args.warmup),
    }
    n_np = results["kernel1_pointcloud"]["numpy"]["mean_ms"]
    n_cp = results["kernel1_pointcloud"]["cupy_wall"]["mean_ms"]
    results["kernel1_pointcloud"]["speedup_numpy_over_cupy"] = round(n_np / n_cp, 3)

    # ---- Kernel 2 (three-way) ----
    fs = FusedSafetyCuPy()
    results["kernel2_fused_safety"] = {
        "original_numpy": time_cpu(lambda: safety_original_numpy(depth_m, fx),
                                   args.iters, args.warmup),
        "optimized_numpy": time_cpu(lambda: safety_optimized_numpy(depth_m, fx),
                                    args.iters, args.warmup),
        "cupy_wall": time_gpu_wall(lambda: fs(depth_m, fx), args.iters, args.warmup),
    }
    s = results["kernel2_fused_safety"]
    s["speedup_orig_over_opt"] = round(
        s["original_numpy"]["mean_ms"] / s["optimized_numpy"]["mean_ms"], 3)
    s["speedup_opt_over_cupy"] = round(
        s["optimized_numpy"]["mean_ms"] / s["cupy_wall"]["mean_ms"], 3)
    s["speedup_orig_over_cupy"] = round(
        s["original_numpy"]["mean_ms"] / s["cupy_wall"]["mean_ms"], 3)

    results["kernel_launch_overhead"] = kernel_launch_overhead()

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))
    print(f"\nWrote {args.out}")


if __name__ == "__main__":
    main()
