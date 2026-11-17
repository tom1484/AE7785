#!/usr/bin/env python3
"""Assemble the Lab 6 submission directory (and optionally the zip).

  python scripts/make_submission.py                       # stage files (no model)
  python scripts/make_submission.py --model submission/model.onnx --requirements --zip Last1_Last2_Lab6.zip

Stages into submission/: model_grader.py, train_model.py, readme.txt, supplemental/{corrupt,preprocess}.py,
model.onnx. Verifies model_grader.py differs from provided/model_grader.py only inside
initialize_model() / predict() (and imports). --requirements runs provided/generate_requirements.py
inside submission/ with this venv's python3 first on PATH.
"""

import argparse
import ast
import os
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

from lab6 import REPO_ROOT

SUB = REPO_ROOT / "submission"
SRC = REPO_ROOT / "src" / "submission"
ALLOWED_FUNCS = {"initialize_model", "predict"}


def grader_skeleton(path: Path) -> str:
    """AST dump with imports and the bodies of the allowed functions removed."""
    tree = ast.parse(path.read_text())
    body = []
    for node in tree.body:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            continue
        if isinstance(node, ast.FunctionDef) and node.name in ALLOWED_FUNCS:
            node = ast.FunctionDef(node.name, node.args, [], [], None, None, [])
        body.append(node)
    return ast.dump(ast.Module(body, []))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", help="ONNX file to include as model.onnx")
    ap.add_argument("--requirements", action="store_true")
    ap.add_argument("--zip", help="zip file name, e.g. Last1_Last2_Lab6.zip (written to outputs/)")
    args = ap.parse_args()

    if grader_skeleton(SRC / "model_grader.py") != grader_skeleton(REPO_ROOT / "provided" / "model_grader.py"):
        sys.exit("model_grader.py changed outside imports/initialize_model/predict")

    SUB.mkdir(exist_ok=True)
    for name in ("model_grader.py", "train_model.py", "readme.txt"):
        shutil.copy(SRC / name, SUB / name)
    sup = SUB / "supplemental"
    sup.mkdir(exist_ok=True)
    (sup / "__init__.py").write_text("")
    for name in ("corrupt.py", "preprocess.py"):
        shutil.copy(REPO_ROOT / "src" / "lab6" / name, sup / name)
    shutil.copy(REPO_ROOT / "provided" / "generate_requirements.py", SUB / "generate_requirements.py")
    if args.model:
        src = Path(args.model).resolve()
        if src != (SUB / "model.onnx").resolve():
            shutil.copy(src, SUB / "model.onnx")

    if args.requirements:
        env = dict(os.environ, PATH=f"{Path(sys.executable).parent}:{os.environ['PATH']}")
        subprocess.run([sys.executable, "generate_requirements.py"], cwd=SUB, env=env, check=True)
        print((SUB / "requirements.txt").read_text())

    if args.zip:
        files = ["model_grader.py", "train_model.py", "readme.txt", "requirements.txt", "model.onnx",
                 "supplemental/__init__.py", "supplemental/corrupt.py", "supplemental/preprocess.py"]
        missing = [f for f in files if not (SUB / f).exists()]
        if missing:
            sys.exit(f"missing: {missing}")
        out = REPO_ROOT / "outputs" / args.zip
        with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
            for f in files:
                z.write(SUB / f, f)
        print("wrote", out, f"{out.stat().st_size / 1e6:.1f} MB")

    print("staged:", sorted(str(p.relative_to(SUB)) for p in SUB.rglob("*") if p.is_file() and "work" not in p.parts))


if __name__ == "__main__":
    main()
