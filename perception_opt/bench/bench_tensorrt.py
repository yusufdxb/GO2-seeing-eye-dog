#!/usr/bin/env python3
"""TensorRT FP16 vs PyTorch FP32 inference latency — RTX 5070 (config 8).

Exports yolov8n to a TensorRT FP16 engine and times the exact inference call
the perception node makes (`model(img)`), against the PyTorch baseline.

SCOPE / HONESTY:
  * RTX 5070 only. The engine that actually ships runs on the Jetson Orin NX
    and MUST be rebuilt there (TensorRT engines are not portable across GPUs);
    Jetson FP16/INT8 export + latency are BLOCKED-HW.
  * mAP validation needs a real GO2 camera test set, which is not available on
    this machine -> BLOCKED-HW. No accuracy numbers are produced here.
  * INT8 needs 500+ real calibration frames -> BLOCKED-HW.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent


def stats(samples_ms):
    a = np.asarray(samples_ms, np.float64)
    return {"n": int(a.size), "mean_ms": float(a.mean()), "std_ms": float(a.std()),
            "p50_ms": float(np.percentile(a, 50)), "p95_ms": float(np.percentile(a, 95)),
            "p99_ms": float(np.percentile(a, 99)), "min_ms": float(a.min())}


def time_infer(model, img, iters, warmup):
    for _ in range(warmup):
        model(img, verbose=False)
    samples = []
    for _ in range(iters):
        t0 = time.perf_counter_ns()
        model(img, verbose=False)
        samples.append((time.perf_counter_ns() - t0) / 1e6)
    return stats(samples)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default=str(Path.home() / "Projects/pythonclass/yolov8n.pt"))
    ap.add_argument("--iters", type=int, default=500)
    ap.add_argument("--warmup", type=int, default=50)
    ap.add_argument("--out", default=str(ROOT / "results" / "bench_tensorrt_rtx5070.json"))
    args = ap.parse_args()

    import torch
    from ultralytics import YOLO

    dev = torch.cuda.get_device_name(0)
    img = (np.random.rand(480, 640, 3) * 255).astype(np.uint8)

    # PyTorch FP32 baseline (CUDA)
    m_pt = YOLO(args.weights)
    m_pt.to("cuda")
    pt = time_infer(m_pt, img, args.iters, args.warmup)

    # Export FP16 engine (cached next to weights)
    eng_path = Path(args.weights).with_suffix(".engine")
    t_exp0 = time.perf_counter()
    exported = m_pt.export(format="engine", half=True, imgsz=640, device=0)
    export_s = time.perf_counter() - t_exp0
    # ultralytics returns the engine path; fall back to convention
    eng = str(exported) if exported else str(eng_path)

    m_trt = YOLO(eng, task="detect")
    trt = time_infer(m_trt, img, args.iters, args.warmup)

    results = {
        "platform": {"device": dev, "note": "RTX 5070 only (config 8). Jetson engine BLOCKED-HW."},
        "blocked_hw": ["mAP validation (no real GO2 test set)",
                       "INT8 calibration (no 500+ real frames)",
                       "Jetson FP16/INT8 engine + latency"],
        "export_seconds": round(export_s, 1),
        "engine_path": eng,
        "pytorch_fp32_cuda": pt,
        "tensorrt_fp16": trt,
        "speedup_pt_over_trt": round(pt["mean_ms"] / trt["mean_ms"], 3),
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))
    print(f"\nWrote {args.out}")


if __name__ == "__main__":
    main()
