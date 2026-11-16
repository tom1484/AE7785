#!/usr/bin/env python3
"""Evaluate exported ONNX models on corrupted copies of a split's val images.

Every model sees the exact same corrupted images (seeded per image x corruption x severity).

Usage:
  python scripts/robustness.py --split holdout-2026F_imgs_seed0 \
      --models yolov8n_cls:224 yolov8n_cls:160 yolov8n_cls:128 yolov8s_cls:224 yolov8m_cls:224
  python scripts/robustness.py --samples   # write a contact sheet of all corruptions only
"""

import argparse
import json
import zlib
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

from lab6 import CLASS_NAMES, REPO_ROOT
from lab6.corrupt import CORRUPTIONS, NAMES, composite
from lab6.data import load_manifest
from lab6.infer import OrtClassifier

REPORTS = REPO_ROOT / "outputs" / "reports"


def rng_for(*keys) -> np.random.Generator:
    return np.random.default_rng(zlib.crc32("|".join(map(str, keys)).encode()))


def contact_sheet(img_path: Path, out: Path):
    img = cv2.imread(str(img_path))
    tw, th = 205, 154
    rows = []
    for name in NAMES:
        tiles = [cv2.resize(img, (tw, th))]
        for s in (1, 2, 3):
            t = cv2.resize(CORRUPTIONS[name](img, s, rng_for("sheet", name, s)), (tw, th))
            tiles.append(t)
        row = np.hstack(tiles)
        cv2.putText(row, name, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
        rows.append(row)
    comp = [cv2.resize(composite(img, rng_for("sheet", "comp", i))[0], (tw, th)) for i in range(4)]
    comp_row = np.hstack(comp)
    cv2.putText(comp_row, "composite", (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
    rows.append(comp_row)
    cv2.imwrite(str(out), np.vstack(rows))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--split", default="holdout-2026F_imgs_seed0", help="split id without _squashNNN")
    ap.add_argument("--models", nargs="+", default=["yolov8n_cls:224", "yolov8n_cls:160"],
                    help="name:imgsz, resolved to outputs/models/<name>__<split>_squash<imgsz>.onnx")
    ap.add_argument("--composite-copies", type=int, default=5)
    ap.add_argument("--seed", default="0")
    ap.add_argument("--samples", action="store_true")
    args = ap.parse_args()

    REPORTS.mkdir(parents=True, exist_ok=True)
    first_split = next((REPO_ROOT / "data" / "splits").glob(f"{args.split}_squash*"))
    val = load_manifest(first_split, "val")
    if args.samples:
        pick = next(s for s in val if s.label == 1)
        contact_sheet(pick.path, REPORTS / "robustness_samples.jpg")
        print("wrote", REPORTS / "robustness_samples.jpg")
        return

    models = {}
    for spec in args.models:
        name, sz = spec.split(":")
        path = REPO_ROOT / "outputs" / "models" / f"{name}__{args.split}_squash{sz}.onnx"
        models[f"{name.replace('_cls', '')}@{sz}"] = OrtClassifier(path, int(sz))

    # records: one row per (condition, image) with each model's prediction
    rows = []
    for idx, s in enumerate(val):
        img = cv2.imread(str(s.path))
        variants = [("clean", 0, img, "")]
        for name in NAMES:
            for sev in (1, 2, 3):
                variants.append((name, sev, CORRUPTIONS[name](img, sev, rng_for(args.seed, s.path.name, name, sev)), ""))
        for c in range(args.composite_copies):
            im, applied = composite(img, rng_for(args.seed, s.path.name, "composite", c))
            variants.append(("composite", c, im, "+".join(applied)))
        for cond, sev, im, applied in variants:
            row = {"image": f"{s.source}/{s.path.name}", "label": s.label, "cond": cond, "sev": sev, "applied": applied}
            for m, clf in models.items():
                row[m] = clf.predict(im)
            rows.append(row)
        if idx % 25 == 0:
            print(f"{idx}/{len(val)} images", flush=True)

    df = pd.DataFrame(rows)
    mcols = list(models)
    correct = df[mcols].eq(df["label"], axis=0)

    summary = {}
    clean = correct[df.cond == "clean"].mean()
    comp = correct[df.cond == "composite"].mean()
    single = correct[~df.cond.isin(["clean", "composite"])]
    per_sev = {sev: correct[(~df.cond.isin(["clean", "composite"])) & (df.sev == sev)].mean() for sev in (1, 2, 3)}
    head = pd.DataFrame({"clean": clean, "single sev1": per_sev[1], "single sev2": per_sev[2],
                         "single sev3": per_sev[3], "single mean": single.mean(), "composite": comp}).T

    per_cond = correct[~df.cond.isin(["clean", "composite"])].groupby(
        [df.cond, df.sev]).mean().unstack("sev")
    per_cond = per_cond.reindex(NAMES)

    # confusion pairs on composite
    comp_df = df[df.cond == "composite"]
    pairs = {}
    for m in mcols:
        wrong = comp_df[comp_df[m] != comp_df.label]
        pairs[m] = (wrong.label.map(CLASS_NAMES) + "->" + wrong[m].map(CLASS_NAMES)).value_counts().head(6).to_dict()

    tag = f"robustness__{args.split}"
    md = [f"# Robustness: {args.split}\n",
          f"val images: {len(val)}; per-corruption: 16 x 3 severities; composite: {args.composite_copies} copies/image\n",
          "## Headline accuracy\n", (head * 100).round(1).to_markdown(), "\n",
          "## Per corruption (accuracy %, columns = model, severity)\n",
          (per_cond * 100).round(1).to_markdown(), "\n",
          "## Top composite confusions (true->pred: count)\n",
          "\n".join(f"- {m}: {p}" for m, p in pairs.items()), "\n"]
    (REPORTS / f"{tag}.md").write_text("\n".join(md))
    df.to_csv(REPORTS / f"{tag}.csv", index=False)
    summary = {"headline": (head * 100).round(2).to_dict(), "confusions": pairs}
    (REPORTS / f"{tag}.json").write_text(json.dumps(summary, indent=1))
    print("\n".join(md))


if __name__ == "__main__":
    main()
