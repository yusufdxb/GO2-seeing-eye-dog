#!/usr/bin/env python3
"""
Speaker Identification Evaluation — GO2 Seeing-Eye Dog
=======================================================
Computes Equal Error Rate (EER) and Detection Error Tradeoff (DET) curve
for the speaker identification component of the identity-gating pipeline.

The speaker ID system uses cosine similarity between speaker embeddings
(x-vector / d-vector) to confirm that the detected voice matches the enrolled owner.

Usage:
  # 1. Enroll speaker (collect 10-30s of enrollment audio):
  python3 evaluation/eval_speaker_id.py enroll \
      --audio ~/data/speaker_id/owner_enrollment.wav \
      --output ~/models/speaker_id/owner_embedding.npy

  # 2. Evaluate EER on test set:
  python3 evaluation/eval_speaker_id.py eval \
      --enrollment ~/models/speaker_id/owner_embedding.npy \
      --test-dir ~/data/speaker_id/test/ \
      --output evaluation/results/speaker_id_eer.json

  # 3. Plot DET curve:
  python3 evaluation/eval_speaker_id.py plot \
      --results evaluation/results/speaker_id_eer.json

Test set structure:
  test/
    target/     ← audio clips from the enrolled speaker (positive trials)
    nontarget/  ← audio clips from other speakers (negative trials)

Dependencies:
  pip install speechbrain numpy scipy matplotlib soundfile tqdm
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np


# ── EER computation ──────────────────────────────────────────────────────────

def compute_eer(
    target_scores: List[float],
    nontarget_scores: List[float],
) -> Tuple[float, float]:
    """
    Compute Equal Error Rate (EER) and the corresponding threshold.

    Args:
        target_scores:    Similarity scores for target (same-speaker) trials.
        nontarget_scores: Similarity scores for non-target (different-speaker) trials.

    Returns:
        (eer: float, threshold: float) where EER is in [0, 1].
    """
    all_scores = sorted(set(target_scores + nontarget_scores))
    best_diff = float("inf")
    best_eer = 0.5
    best_threshold = 0.0

    for threshold in all_scores:
        # False Rejection Rate: target trials rejected
        frr = sum(1 for s in target_scores if s < threshold) / len(target_scores)
        # False Acceptance Rate: non-target trials accepted
        far = sum(1 for s in nontarget_scores if s >= threshold) / len(nontarget_scores)

        diff = abs(frr - far)
        candidate = (frr + far) / 2
        # Keep the operating point closest to frr==far; break ties by lowest EER
        if diff < best_diff or (diff == best_diff and candidate < best_eer):
            best_diff = diff
            best_eer = candidate
            best_threshold = threshold

    return best_eer, best_threshold


def compute_det_curve(
    target_scores: List[float],
    nontarget_scores: List[float],
    n_points: int = 100,
) -> Tuple[List[float], List[float]]:
    """
    Compute FAR and FRR at each threshold for DET curve plotting.

    Returns:
        (far_list, frr_list) — each of length n_points
    """
    min_score = min(target_scores + nontarget_scores)
    max_score = max(target_scores + nontarget_scores)
    thresholds = [min_score + i * (max_score - min_score) / n_points
                  for i in range(n_points + 1)]

    far_list, frr_list = [], []
    for thr in thresholds:
        frr = sum(1 for s in target_scores if s < thr) / len(target_scores)
        far = sum(1 for s in nontarget_scores if s >= thr) / len(nontarget_scores)
        far_list.append(far)
        frr_list.append(frr)

    return far_list, frr_list


# ── Embedding extraction ─────────────────────────────────────────────────────

def extract_embedding(audio_path: str) -> np.ndarray:
    """
    Extract a speaker embedding (d-vector) from an audio file using SpeechBrain.

    Requires: pip install speechbrain
    Model: speechbrain/spkrec-ecapa-voxceleb (ECAPA-TDNN, pre-trained on VoxCeleb)

    Args:
        audio_path: Path to a .wav file (any sample rate — resampled internally).

    Returns:
        Normalized L2 embedding vector of shape (192,).
    """
    try:
        import torch
        import torchaudio
        from speechbrain.pretrained import EncoderClassifier

        model = EncoderClassifier.from_hparams(
            source="speechbrain/spkrec-ecapa-voxceleb",
            savedir="/tmp/speechbrain_ecapa",
        )
        signal, fs = torchaudio.load(audio_path)
        if fs != 16000:
            resampler = torchaudio.transforms.Resample(fs, 16000)
            signal = resampler(signal)
        with torch.no_grad():
            embedding = model.encode_batch(signal)
        emb = embedding.squeeze().numpy()
        return emb / np.linalg.norm(emb)

    except ImportError:
        raise ImportError(
            "speechbrain not installed. Run: pip install speechbrain torchaudio"
        )


def cosine_similarity(a: np.ndarray, b: np.ndarray) -> float:
    """Cosine similarity between two L2-normalized embedding vectors."""
    return float(np.dot(a, b))


# ── CLI commands ─────────────────────────────────────────────────────────────

def cmd_enroll(args):
    """Enroll a speaker from an audio file."""
    print(f"Extracting enrollment embedding from: {args.audio}")
    embedding = extract_embedding(args.audio)
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.save(str(out_path), embedding)
    print(f"Enrollment embedding saved: {out_path} (shape={embedding.shape})")


def cmd_eval(args):
    """Evaluate EER against a test set directory."""
    from tqdm import tqdm

    enrollment = np.load(args.enrollment)
    test_dir = Path(args.test_dir)

    target_dir = test_dir / "target"
    nontarget_dir = test_dir / "nontarget"

    if not target_dir.exists() or not nontarget_dir.exists():
        raise FileNotFoundError(
            f"Expected {target_dir} and {nontarget_dir}. "
            "See script docstring for test set structure."
        )

    target_files = list(target_dir.glob("*.wav"))
    nontarget_files = list(nontarget_dir.glob("*.wav"))

    print(f"Target trials:    {len(target_files)}")
    print(f"Non-target trials: {len(nontarget_files)}")

    target_scores = []
    for f in tqdm(target_files, desc="Target"):
        emb = extract_embedding(str(f))
        target_scores.append(cosine_similarity(enrollment, emb))

    nontarget_scores = []
    for f in tqdm(nontarget_files, desc="Non-target"):
        emb = extract_embedding(str(f))
        nontarget_scores.append(cosine_similarity(enrollment, emb))

    eer, threshold = compute_eer(target_scores, nontarget_scores)
    far_list, frr_list = compute_det_curve(target_scores, nontarget_scores)

    results = {
        "eer": eer,
        "eer_threshold": threshold,
        "n_target_trials": len(target_scores),
        "n_nontarget_trials": len(nontarget_scores),
        "mean_target_score": float(np.mean(target_scores)),
        "mean_nontarget_score": float(np.mean(nontarget_scores)),
        "det_curve": {"far": far_list, "frr": frr_list},
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(results, f, indent=2)

    print(f"\nEER: {eer*100:.2f}%  (threshold={threshold:.4f})")
    print(f"Mean target score:    {results['mean_target_score']:.4f}")
    print(f"Mean non-target score: {results['mean_nontarget_score']:.4f}")
    print(f"Results saved: {out_path}")


def cmd_plot(args):
    """Plot DET curve from saved results JSON."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        raise ImportError("matplotlib not installed. Run: pip install matplotlib")

    with open(args.results) as f:
        results = json.load(f)

    far = results["det_curve"]["far"]
    frr = results["det_curve"]["frr"]
    eer = results["eer"]

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(far, frr, "b-", linewidth=2, label="DET curve")
    ax.plot([0, 1], [0, 1], "k--", alpha=0.4, label="Chance")
    ax.scatter([eer], [eer], color="red", zorder=5, label=f"EER = {eer*100:.1f}%")
    ax.set_xlabel("False Acceptance Rate (FAR)")
    ax.set_ylabel("False Rejection Rate (FRR)")
    ax.set_title("Speaker ID — Detection Error Tradeoff")
    ax.legend()
    ax.grid(alpha=0.3)

    out_path = Path(args.results).with_suffix(".png")
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"DET curve saved: {out_path}")
    plt.show()


# ── Entry point ──────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Speaker ID evaluation")
    sub = parser.add_subparsers(dest="command", required=True)

    enroll_p = sub.add_parser("enroll", help="Enroll a speaker from audio")
    enroll_p.add_argument("--audio", required=True)
    enroll_p.add_argument("--output", default="~/models/speaker_id/owner_embedding.npy")

    eval_p = sub.add_parser("eval", help="Evaluate EER on test set")
    eval_p.add_argument("--enrollment", required=True)
    eval_p.add_argument("--test-dir", required=True)
    eval_p.add_argument("--output", default="evaluation/results/speaker_id_eer.json")

    plot_p = sub.add_parser("plot", help="Plot DET curve")
    plot_p.add_argument("--results", required=True)

    args = parser.parse_args()
    if args.command == "enroll":
        cmd_enroll(args)
    elif args.command == "eval":
        cmd_eval(args)
    elif args.command == "plot":
        cmd_plot(args)


if __name__ == "__main__":
    main()
