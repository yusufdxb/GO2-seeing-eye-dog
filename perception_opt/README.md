# Perception Pipeline Optimization

Profiling-driven GPU acceleration of the GO2 seeing-eye-dog ROS 2 RGB-D
perception pipeline. Measure first, optimize the measured bottleneck, report
every number honestly (including where the GPU does *not* help).

> Status: **software layer complete and verified on RTX 5070 (Blackwell, sm_120).**
> Jetson Orin NX 16GB baselines, thermal characterization, on-device TensorRT
> engines, INT8 calibration, and mAP validation are **BLOCKED-HW** — they need
> the physical Jetson + GO2 camera and are owed at the next CaresLab session.

## Layout

```
perception_opt/
├── kernels/
│   ├── depth_to_pointcloud.py   # Kernel 1: NumPy ref + CuPy RawKernel
│   └── fused_safety.py          # Kernel 2: original/optimized NumPy + CuPy
├── tests/test_kernels_parity.py # CuPy-vs-NumPy parity (auto-skips w/o GPU)
├── bench/
│   ├── run_bench.py             # three-way kernel benchmark (config 8)
│   └── bench_tensorrt.py        # YOLOv8 TRT FP16 vs PyTorch latency (config 8)
└── results/                     # committed raw JSON
```

ROS 2 instrumentation (Phase 0) lives in the packages themselves:
`go2_msgs/msg/PipelineMetrics.msg`, per-stage `perf_counter_ns` timing in
`perception_node.py` and `safety_monitor_node.py`, and the
`pipeline_metrics_logger` node that writes per-frame CSV.

## Reproduce (RTX 5070)

```bash
pip install --user cupy-cuda12x tensorrt-cu12==10.13.*
# Blackwell + CUDA 12.8 driver: remove any CUDA-13 nvrtc wheel or RawKernel JIT
# fails with CUDA_ERROR_INVALID_IMAGE (see BENCHMARKS.md "Environment notes").
python3 -m pytest perception_opt/tests/ -v
python3 perception_opt/bench/run_bench.py
python3 perception_opt/bench/bench_tensorrt.py
```

## Headline results (RTX 5070, 1000 frames, 100-frame warmup)

| Optimization | Baseline | Optimized | Speedup | Verdict |
|---|---|---|---|---|
| Depth→pointcloud (307k px) | NumPy 2.25 ms | CuPy 0.050 ms | **45×** | GPU wins decisively |
| Safety: vectorize Python loop | orig 0.613 ms | opt NumPy 0.191 ms | **3.2×** | most of the gain is here |
| Safety: NumPy→CuPy | opt 0.191 ms | CuPy 0.186 ms | **1.03×** | GPU barely helps; documented |
| YOLOv8n inference | PyTorch 2.18 ms | TRT FP16 1.39 ms | **1.57×** | +5× tighter jitter (p99 2.74→1.52 ms) |

The safety result is the honest finding the project exists to surface: porting a
small-ROI, reduction-heavy, sequential-scan workload to CUDA buys almost nothing
once the Python `for` loop is vectorized. The 307k-pixel back-projection is the
opposite — embarrassingly parallel, no reductions, and the GPU wins by 45×.

See `BENCHMARKS.md` for full methodology, percentile tables, and BLOCKED-HW items.
