#!/usr/bin/env python3
"""Collect outputs/reports/*.json into one table (printed + outputs/reports/summary.md)."""

import json

import pandas as pd

from lab6 import REPO_ROOT


def main():
    rows = []
    for p in sorted((REPO_ROOT / "outputs" / "reports").glob("*.json")):
        r = json.loads(p.read_text())
        cv2_eval = r["eval"]["cv2"]
        cm = cv2_eval["confusion"]
        rows.append({
            "run": r["run_id"],
            "val_n": cv2_eval["n"],
            "acc_cv2": round(cv2_eval["accuracy"], 4),
            "acc_ort": round(r["eval"]["ort"]["accuracy"], 4),
            "left<->right": cm[1][2] + cm[2][1],
            "errors": len(cv2_eval["errors"]),
            "onnx_mb": round(r["onnx_mb"], 1),
            "cpu_ms_cv2": round(cv2_eval["latency_ms_median"], 1),
            "train_min": round(r["train_minutes"], 1),
            "parity_max_diff": max(v["max_abs_diff"] for v in r["parity"].values()),
        })
    df = pd.DataFrame(rows)
    print(df.to_string(index=False))
    (REPO_ROOT / "outputs" / "reports" / "summary.md").write_text(df.to_markdown(index=False))


if __name__ == "__main__":
    main()
