"""Read course label files, split them, and write ultralytics ImageFolder trees."""

from __future__ import annotations

import csv
import json
import shutil
from dataclasses import dataclass
from pathlib import Path

import cv2
from sklearn.model_selection import train_test_split

from lab6 import CLASS_NAMES, REPO_ROOT
from lab6.preprocess import prepare_image


@dataclass(frozen=True)
class Sample:
    path: Path
    label: int
    source: str  # folder name, e.g. "2026F_imgs"


def read_labels(folder: str | Path) -> list[Sample]:
    folder = REPO_ROOT / folder
    samples = []
    with open(folder / "labels.txt") as f:
        for row in csv.reader(f):
            if not row:
                continue
            name, label = row[0].strip(), int(row[1])
            samples.append(Sample(folder / f"{name}.png", label, folder.name))
    return samples


def class_dir(label: int) -> str:
    # Single-digit prefix keeps ultralytics' alphabetical class order == label id.
    return f"{label}_{CLASS_NAMES[label]}"


def make_split(data_cfg: dict) -> tuple[list[Sample], list[Sample]]:
    samples = [s for d in data_cfg["raw_dirs"] for s in read_labels(d)]
    split = data_cfg["split"]
    if split["strategy"] == "stratified":
        strata = [f"{s.source}/{s.label}" for s in samples]  # balance labels within each folder
        train, val = train_test_split(
            samples, test_size=split["val_ratio"], stratify=strata, random_state=split["seed"]
        )
    elif split["strategy"] == "holdout_folder":
        held = Path(split["holdout_folder"]).name
        train = [s for s in samples if s.source != held]
        val = [s for s in samples if s.source == held]
    else:
        raise ValueError(f"unknown split strategy {split['strategy']!r}")
    return train, val


def split_id(cfg: dict) -> str:
    sp, pp = cfg["data"]["split"], cfg["preprocess"]
    tag = sp["strategy"] if sp["strategy"] == "stratified" else f"holdout-{Path(sp['holdout_folder']).name}"
    return f"{tag}_seed{sp['seed']}_{pp['resize_mode']}{pp['imgsz']}"


def build_split(cfg: dict, force: bool = False) -> Path:
    """Write data/splits/<id>/{train,val}/<cls>/*.png plus manifest.json; return the root."""
    root = REPO_ROOT / "data" / "splits" / split_id(cfg)
    if root.exists() and not force:
        return root
    if root.exists():
        shutil.rmtree(root)
    train, val = make_split(cfg["data"])
    pp = cfg["preprocess"]
    manifest = {}
    for part, items in (("train", train), ("val", val)):
        for label in CLASS_NAMES:
            (root / part / class_dir(label)).mkdir(parents=True, exist_ok=True)
        for s in items:
            img = cv2.imread(str(s.path))
            if img is None:
                raise FileNotFoundError(s.path)
            out = root / part / class_dir(s.label) / f"{s.source}__{s.path.stem}.png"
            cv2.imwrite(str(out), prepare_image(img, pp["imgsz"], pp["resize_mode"]))
        manifest[part] = [
            {"path": str(s.path.relative_to(REPO_ROOT)), "label": s.label, "source": s.source}
            for s in items
        ]
    (root / "manifest.json").write_text(json.dumps(manifest, indent=1))
    return root


def load_manifest(root: Path, part: str = "val") -> list[Sample]:
    data = json.loads((root / "manifest.json").read_text())[part]
    return [Sample(REPO_ROOT / d["path"], d["label"], d["source"]) for d in data]
