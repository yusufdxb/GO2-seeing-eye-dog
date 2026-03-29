#!/usr/bin/env python3
"""
Synthetic EER Evaluation — GO2 Seeing-Eye Dog Speaker ID
=========================================================
Runs the EER pipeline from eval_speaker_id.py on synthetic speaker
embeddings. This validates the evaluation infrastructure end-to-end
without requiring real audio data or SpeechBrain.

The synthetic embeddings model realistic ECAPA-TDNN behaviour:
  - Same-speaker (target) pairs: high cosine similarity, Gaussian noise
  - Different-speaker (nontarget) pairs: low cosine similarity

Results are written to evaluation/results/synthetic_eer.json and
a DET curve is saved to evaluation/results/det_curve_synthetic.png.

Run from repo root:
  python3 evaluation/run_synthetic_eer.py

Dependencies: numpy matplotlib scipy (no speechbrain required)
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

# ── Import EER functions from the main evaluator ─────────────────────────────
sys.path.insert(0, str(Path(__file__).parent))
from eval_speaker_id import compute_det_curve, compute_eer

# ── Matplotlib (headless) ────────────────────────────────────────────────────
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False


# ── Synthetic embedding generator ────────────────────────────────────────────

def make_speaker_embedding(rng: np.random.Generator, dim: int = 192) -> np.ndarray:
    """Generate a random unit-norm embedding for a new speaker identity."""
    v = rng.standard_normal(dim)
    return v / np.linalg.norm(v)


def sample_target_score(
    enrollment: np.ndarray,
    rng: np.random.Generator,
    within_speaker_std: float = 0.05,
) -> float:
    """
    Simulate a same-speaker trial: add small perturbation to enrollment,
    then compute cosine similarity.
    """
    noise = rng.standard_normal(enrollment.shape) * within_speaker_std
    trial = enrollment + noise
    trial = trial / np.linalg.norm(trial)
    return float(np.dot(enrollment, trial))


def sample_nontarget_score(
    enrollment: np.ndarray,
    other_embedding: np.ndarray,
) -> float:
    """
    Simulate a different-speaker trial: cosine similarity between two
    independent random unit vectors.
    """
    return float(np.dot(enrollment, other_embedding))


def generate_trial_scores(
    n_speakers: int = 50,
    n_target_per_speaker: int = 20,
    n_nontarget_per_speaker: int = 20,
    dim: int = 192,
    seed: int = 42,
) -> tuple[list[float], list[float]]:
    """
    Generate realistic target and nontarget cosine similarity scores.

    - n_speakers speakers, each with one enrollment embedding
    - n_target_per_speaker same-speaker trials per speaker
    - n_nontarget_per_speaker different-speaker trials per speaker
    """
    rng = np.random.default_rng(seed)
    target_scores: list[float] = []
    nontarget_scores: list[float] = []

    # Generate all speaker embeddings
    speakers = [make_speaker_embedding(rng, dim) for _ in range(n_speakers)]

    for i, enroll in enumerate(speakers):
        # Target trials (same speaker)
        for _ in range(n_target_per_speaker):
            target_scores.append(sample_target_score(enroll, rng))

        # Nontarget trials (different speaker)
        nontarget_indices = [j for j in range(n_speakers) if j != i]
        chosen = rng.choice(nontarget_indices, size=n_nontarget_per_speaker, replace=True)
        for j in chosen:
            nontarget_scores.append(sample_nontarget_score(enroll, speakers[j]))

    return target_scores, nontarget_scores


def plot_det_curve(
    far: list[float],
    frr: list[float],
    eer: float,
    threshold: float,
    out_path: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot([f * 100 for f in far], [f * 100 for f in frr],
            "b-", linewidth=2, label="DET curve (synthetic)")
    ax.plot([eer * 100], [eer * 100], "ro", markersize=8,
            label=f"EER = {eer*100:.1f}% (threshold={threshold:.3f})")
    ax.plot([0, 100], [0, 100], "k--", linewidth=0.7, alpha=0.4, label="Chance")
    ax.set_xlabel("False Acceptance Rate (%)")
    ax.set_ylabel("False Rejection Rate (%)")
    ax.set_title("Speaker ID — DET Curve (Synthetic Embeddings)")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, 50)
    ax.set_ylim(0, 50)
    fig.tight_layout()
    fig.savefig(str(out_path), dpi=150)
    plt.close(fig)
    print(f"DET curve saved: {out_path}")


def main() -> None:
    print("GO2 Seeing-Eye Dog — Synthetic Speaker ID EER Evaluation")
    print("=" * 58)
    print(
        "\nNote: Using synthetic ECAPA-TDNN-like embeddings (dim=192).\n"
        "For real hardware results, collect audio and run:\n"
        "  python3 evaluation/eval_speaker_id.py eval ...\n"
    )

    results_dir = Path(__file__).parent / "results"
    results_dir.mkdir(exist_ok=True)

    print("Generating trial scores (50 speakers, 20 target + 20 nontarget each)...")
    t0 = time.perf_counter()
    target_scores, nontarget_scores = generate_trial_scores(
        n_speakers=50,
        n_target_per_speaker=20,
        n_nontarget_per_speaker=20,
        dim=192,
        seed=42,
    )
    elapsed = time.perf_counter() - t0

    print(f"  {len(target_scores)} target trials, {len(nontarget_scores)} nontarget trials")
    print(f"  Generated in {elapsed*1000:.1f} ms")

    print("\nComputing EER...")
    eer, threshold = compute_eer(target_scores, nontarget_scores)
    far, frr = compute_det_curve(target_scores, nontarget_scores)

    print(f"  EER:       {eer*100:.2f}%")
    print(f"  Threshold: {threshold:.4f}")

    # Score distribution stats
    t_mean = float(np.mean(target_scores))
    t_std = float(np.std(target_scores))
    nt_mean = float(np.mean(nontarget_scores))
    nt_std = float(np.std(nontarget_scores))

    print("\nScore distributions:")
    print(f"  Target:    mean={t_mean:.4f}  std={t_std:.4f}")
    print(f"  Nontarget: mean={nt_mean:.4f}  std={nt_std:.4f}")

    # Save results
    results = {
        "evaluation_type": "synthetic",
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "config": {
            "n_speakers": 50,
            "n_target_per_speaker": 20,
            "n_nontarget_per_speaker": 20,
            "embedding_dim": 192,
            "seed": 42,
            "note": "Synthetic embeddings modelling ECAPA-TDNN behavior",
        },
        "results": {
            "eer_percent": round(eer * 100, 2),
            "eer_threshold": round(threshold, 4),
            "n_target_trials": len(target_scores),
            "n_nontarget_trials": len(nontarget_scores),
            "target_score_mean": round(t_mean, 4),
            "target_score_std": round(t_std, 4),
            "nontarget_score_mean": round(nt_mean, 4),
            "nontarget_score_std": round(nt_std, 4),
        },
        "next_steps": [
            "Collect 10-30s enrollment audio from target speaker",
            "Collect 20+ target test clips (same speaker, varied conditions)",
            "Collect 20+ nontarget clips (different speakers)",
            "Run: python3 evaluation/eval_speaker_id.py enroll ...",
            "Run: python3 evaluation/eval_speaker_id.py eval ...",
        ],
    }

    json_out = results_dir / "synthetic_eer.json"
    with open(json_out, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nResults written: {json_out}")

    if HAS_MPL:
        det_out = results_dir / "det_curve_synthetic.png"
        plot_det_curve(far, frr, eer, threshold, det_out)
    else:
        print("(matplotlib not available — skipping DET curve plot)")


if __name__ == "__main__":
    main()
