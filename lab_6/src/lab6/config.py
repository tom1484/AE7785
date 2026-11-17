"""YAML experiment configs with single-parent `inherit:` deep-merge."""

from __future__ import annotations

import copy
from pathlib import Path

import yaml


def _deep_merge(base: dict, override: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def load_config(path: str | Path) -> dict:
    path = Path(path).resolve()
    cfg = yaml.safe_load(path.read_text()) or {}
    parent = cfg.pop("inherit", None)
    if parent:
        cfg = _deep_merge(load_config(path.parent / parent), cfg)
    cfg.setdefault("name", path.stem)
    return cfg
