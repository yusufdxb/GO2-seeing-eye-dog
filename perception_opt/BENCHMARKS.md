# Benchmarks & Methodology

All numbers below were **measured on this machine** (mewtwo, RTX 5070, Blackwell
sm_120, driver 12.8). Raw JSON is in `results/`. Nothing here is estimated.
Items requiring the physical Jetson Orin NX 16GB or the GO2 camera are listed
explicitly as **BLOCKED-HW**; no Jetson or accuracy numbers are invented.

## Protocol

- 100-frame warmup discarded before every measurement window.
- 1000 frames per configuration (kernels); 500 per config for YOLO inference.
- `cp.cuda.runtime.deviceSynchronize()` before every GPU timer stop.
- GPU wall time **includes** host↔device transfer — the real pipeline cost on a
  discrete GPU, not a kernel-only figure.
- Reported: mean, std, p50, p95, p99 (never a single best sample).
- Synthetic 640×480 depth with 10% invalid pixels + a floor ramp (deterministic
  seed). Real GO2 frames are BLOCKED-HW; synthetic input is used only for
  *latency* (shape/dtype identical to the camera), never for accuracy.

## Kernel 1 — Depth → Pointcloud (RTX 5070, config 8)

307,200 pixels, one CUDA thread each, pinhole back-projection. Parity vs NumPy:
max abs diff < 1e-4 on points, exact on valid mask (4 tests pass).

| Impl | mean | p50 | p95 | p99 |
|---|---|---|---|---|
| NumPy | 2.25 ms | 2.24 ms | 2.32 ms | 2.45 ms |
| CuPy (wall, incl. transfer) | 0.050 ms | 0.050 ms | 0.052 ms | 0.061 ms |

**Speedup: 45×.** Embarrassingly parallel, no reductions → GPU wins decisively
even with transfer included.

## Kernel 2 — Fused Safety (RTX 5070, config 8) — three-way

Single fused CuPy kernel (atomics over the full frame) + trivial 640-column
finalize. Parity: CuPy matches optimized NumPy to 2e-3 (test passes). The
optimized/CuPy paths use **mean/min** where the original node uses
**median/5th-percentile** — a documented *semantic* change (median/percentile
need a sort, which does not fuse into a streaming pass), asserted in the tests so
it is never mistaken for a bug.

| Impl | mean | p50 | p95 | p99 |
|---|---|---|---|---|
| Original NumPy (percentile/median/`for`-loop) | 0.613 ms | 0.607 ms | 0.645 ms | 0.777 ms |
| Optimized NumPy (vectorized) | 0.191 ms | 0.189 ms | 0.197 ms | 0.228 ms |
| CuPy fused (wall, incl. transfer) | 0.186 ms | 0.184 ms | 0.212 ms | 0.245 ms |

- original → optimized: **3.2×** (vectorization + semantic change)
- optimized → CuPy: **1.03×** (pure parallelization)
- Kernel launch overhead: **2.6 µs** mean (4.9 µs p99)

**Honest finding:** for the safety workload, nearly all the speedup comes from
deleting the Python `for`-loop, not from CUDA. Atomics + small ROIs + a
sequential longest-run scan + a host finalize + transfer leave the GPU only
marginally ahead of vectorized NumPy on a discrete card. This is the negative
result the project was designed to surface.

## YOLOv8n — TensorRT FP16 vs PyTorch FP32 (RTX 5070, config 8)

Engine export took 249 s on the 5070. 500 frames, 50-frame warmup.

| Impl | mean | p50 | p95 | p99 | std |
|---|---|---|---|---|---|
| PyTorch FP32 (CUDA) | 2.18 ms | 2.12 ms | 2.50 ms | 2.74 ms | 0.149 ms |
| TensorRT FP16 | 1.39 ms | 1.38 ms | 1.42 ms | 1.52 ms | 0.029 ms |

**Speedup: 1.57×** on the RTX 5070, plus a **5× reduction in jitter** (std
0.149 → 0.029 ms; p99 2.74 → 1.52 ms). The modest mean speedup is expected:
yolov8n is tiny and a discrete Blackwell GPU is far from saturated, so kernel
launch / framework overhead dominates and TensorRT's main win here is
determinism, not throughput. The project's >2× inference target is a **Jetson**
criterion (weaker GPU, where TRT's win is larger) and remains BLOCKED-HW. The
tighter p99 is the safety-relevant result: less worst-case latency jitter.

## Environment notes (Blackwell + CUDA 12.8 driver)

- `cupy-cuda12x` 14.1.1 bundles CUDA-12.9 NVRTC and pulls CUDA-13 nvrtc wheels;
  on a 12.8 driver this makes both prebuilt kernels **and** RawKernel JIT fail
  with `CUDA_ERROR_INVALID_IMAGE`. Fix: `pip uninstall nvidia-cuda-nvrtc
  nvidia-cuda-runtime` (the CUDA-13 wheels). RawKernel JIT then works; the
  prebuilt elementwise kernels (`cp.arange().sum()`) may still fail, which is why
  the kernels here are all RawKernel.
- TensorRT: install `tensorrt-cu12==10.13.*` (the default `tensorrt` pulls a
  cu13 build that will not load on the 12.8 driver).

## BLOCKED-HW (owed at CaresLab / on the Jetson)

- 1000-frame baselines on Jetson at 15W and 25W (Phase 0)
- nsys trace + 30-min tegrastats thermal characterization (Phase 1)
- TensorRT FP16/INT8 engines built **on the Jetson** (engines are not portable)
- INT8 calibration from 500+ real GO2 frames
- mAP@0.5 / mAP@0.5:0.95 validation (FP32 vs FP16 vs INT8) on a real test set
- CuPy kernels on Jetson unified memory + cross-platform analysis
- Ablation configs 1–7 (all Jetson)
- PointCloud2 in RViz on the live stream
