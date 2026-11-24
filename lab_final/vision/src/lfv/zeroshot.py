"""Zero-shot (text-prompted) detectors used to bootstrap sign boxes.

Each detector is a callable: bgr image -> list of (x0, y0, x1, y1, score), best first,
in source-image pixels. Benchmarked in scripts/bench_zeroshot.py (see .ai/PROJECT.md).
"""

from __future__ import annotations

import cv2

PROMPTS = ['arrow', 'stop sign', 'star', 'u-turn arrow sign']

Box = tuple[float, float, float, float, float]


def owlv2(device: str = 'mps', name: str = 'google/owlv2-base-patch16-ensemble', top_k: int = 3):
    import torch
    from transformers import Owlv2ForObjectDetection, Owlv2Processor

    proc = Owlv2Processor.from_pretrained(name)
    model = Owlv2ForObjectDetection.from_pretrained(name).to(device).eval()

    def run(bgr) -> list[Box]:
        h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        inp = proc(text=[PROMPTS], images=rgb, return_tensors='pt').to(device)
        with torch.no_grad():
            out = model(**inp)
        # OWLv2 pads to a square; boxes are relative to the padded size.
        s = max(h, w)
        res = proc.post_process_grounded_object_detection(
            out, threshold=0.0, target_sizes=torch.tensor([[s, s]]))[0]
        return _top(res['boxes'], res['scores'], w, h, top_k)
    return run


def grounding_dino(device: str = 'mps', name: str = 'IDEA-Research/grounding-dino-tiny',
                   top_k: int = 3):
    import torch
    from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

    proc = AutoProcessor.from_pretrained(name)
    model = AutoModelForZeroShotObjectDetection.from_pretrained(name).to(device).eval()
    text = '. '.join(PROMPTS) + '.'

    def run(bgr) -> list[Box]:
        h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        inp = proc(images=rgb, text=text, return_tensors='pt').to(device)
        with torch.no_grad():
            out = model(**inp)
        res = proc.post_process_grounded_object_detection(
            out, inp.input_ids, threshold=0.05, text_threshold=0.05, target_sizes=[(h, w)])[0]
        return _top(res['boxes'], res['scores'], w, h, top_k)
    return run


def _top(boxes, scores, w, h, k) -> list[Box]:
    order = scores.argsort(descending=True)[:k]
    out = []
    for i in order.tolist():
        x0, y0, x1, y1 = boxes[i].tolist()
        out.append((max(0.0, x0), max(0.0, y0), min(float(w), x1), min(float(h), y1),
                    float(scores[i])))
    return out


def iou(a, b) -> float:
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0
