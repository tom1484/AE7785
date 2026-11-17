"""Label-preserving corruptions of raw camera frames for robustness evaluation.

Every function: (img_bgr_uint8, severity in {1,2,3}, rng) -> img_bgr_uint8 of the same size.
Applied to the ORIGINAL frame, before lab6.preprocess, so the full predict() path is exercised.
Excluded on purpose: horizontal flip (swaps left/right), vertical flip, rotation > 10 deg,
large hue shifts (would change the sign's color class).
"""

from __future__ import annotations

import cv2
import numpy as np

BORDER = cv2.BORDER_REPLICATE


def _clip(x):
    return np.clip(x, 0, 255).astype(np.uint8)


def _sev(severity, values):
    return values[severity - 1]


# --- Lighting & color -------------------------------------------------------------------------

def brightness(img, s, rng):
    if rng.random() < 0.7:  # darkening is the realistic failure (dim corners of the maze)
        gain, gamma = _sev(s, [(0.65, 1.2), (0.5, 1.5), (0.35, 1.8)])
    else:
        gain, gamma = _sev(s, [(1.15, 0.9), (1.25, 0.8), (1.4, 0.7)])
    x = 255.0 * (img / 255.0) ** gamma * gain
    return _clip(x)


def contrast(img, s, rng):
    k = _sev(s, [0.8, 0.6, 0.4])
    mean = img.reshape(-1, 3).mean(0)
    return _clip(mean + k * (img.astype(np.float32) - mean))


def uneven_light(img, s, rng):
    h, w = img.shape[:2]
    low = _sev(s, [0.7, 0.55, 0.4])
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    if rng.random() < 0.5:  # linear gradient in a random direction
        a = rng.uniform(0, 2 * np.pi)
        t = (xx - w / 2) * np.cos(a) + (yy - h / 2) * np.sin(a)
        t = (t - t.min()) / (t.max() - t.min())
    else:  # vignette
        r = np.sqrt(((xx - w / 2) / (w / 2)) ** 2 + ((yy - h / 2) / (h / 2)) ** 2)
        t = 1 - np.clip(r / r.max(), 0, 1)
    gain = low + (1 - low) * t
    return _clip(img * gain[..., None])


def white_balance(img, s, rng):
    d = _sev(s, [0.10, 0.18, 0.25])
    sign = rng.choice([-1, 1])  # warm (+R -B) or cool (-R +B)
    gains = np.array([1 - sign * d, 1 + rng.uniform(-d, d) / 3, 1 + sign * d])  # B, G, R
    return _clip(img * gains)


def desaturate(img, s, rng):
    k = _sev(s, [0.8, 0.6, 0.4])
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.float32)
    hsv[..., 1] *= k
    return cv2.cvtColor(_clip(hsv), cv2.COLOR_HSV2BGR)


# --- Sensor & stream ----------------------------------------------------------------------------

def gaussian_noise(img, s, rng):
    sigma = _sev(s, [5, 12, 20])
    return _clip(img + rng.normal(0, sigma, img.shape))


def jpeg(img, s, rng):
    q = _sev(s, [40, 20, 10])
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, q])
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def motion_blur(img, s, rng):
    k = _sev(s, [5, 9, 15])
    kernel = np.zeros((k, k), np.float32)
    kernel[k // 2, :] = 1.0
    rot = cv2.getRotationMatrix2D((k / 2 - 0.5, k / 2 - 0.5), rng.uniform(0, 180), 1.0)
    kernel = cv2.warpAffine(kernel, rot, (k, k))
    kernel /= kernel.sum()
    return cv2.filter2D(img, -1, kernel, borderType=BORDER)


def defocus(img, s, rng):
    sigma = _sev(s, [1.0, 2.0, 3.0])
    return cv2.GaussianBlur(img, (0, 0), sigma, borderType=BORDER)


def low_res(img, s, rng):
    f = _sev(s, [1 / 2, 1 / 3, 1 / 4])
    h, w = img.shape[:2]
    small = cv2.resize(img, (max(1, int(w * f)), max(1, int(h * f))), interpolation=cv2.INTER_AREA)
    return cv2.resize(small, (w, h), interpolation=cv2.INTER_LINEAR)


# --- Geometry -----------------------------------------------------------------------------------

def perspective(img, s, rng):
    """Approximate a yaw of ~10/20/30 deg: one vertical edge shrinks toward the center."""
    h, w = img.shape[:2]
    d = _sev(s, [0.06, 0.12, 0.18]) * h
    src = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    if rng.random() < 0.5:  # left edge farther away
        dst = np.float32([[0, d], [w, 0], [w, h], [0, h - d]])
    else:
        dst = np.float32([[0, 0], [w, d], [w, h - d], [0, h]])
    M = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(img, M, (w, h), borderMode=BORDER)


def rotate(img, s, rng):
    deg = _sev(s, [3, 6, 10]) * rng.choice([-1, 1])
    h, w = img.shape[:2]
    M = cv2.getRotationMatrix2D((w / 2, h / 2), deg, 1.0)
    return cv2.warpAffine(img, M, (w, h), borderMode=BORDER)


def translate(img, s, rng):
    f = _sev(s, [0.05, 0.12, 0.20])
    h, w = img.shape[:2]
    a = rng.uniform(0, 2 * np.pi)
    M = np.float32([[1, 0, f * w * np.cos(a)], [0, 1, f * h * np.sin(a)]])
    return cv2.warpAffine(img, M, (w, h), borderMode=BORDER)


def zoom(img, s, rng):
    z = _sev(s, [0.9, 0.8, 0.7]) if rng.random() < 0.5 else _sev(s, [1.1, 1.2, 1.3])
    h, w = img.shape[:2]
    M = cv2.getRotationMatrix2D((w / 2, h / 2), 0, z)
    return cv2.warpAffine(img, M, (w, h), borderMode=BORDER)


# --- Occlusion & glare --------------------------------------------------------------------------

def glare(img, s, rng):
    h, w = img.shape[:2]
    area = _sev(s, [0.03, 0.06, 0.10]) * h * w
    r = np.sqrt(area / np.pi)
    cx, cy = rng.uniform(0.2, 0.8) * w, rng.uniform(0.2, 0.8) * h
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    alpha = np.exp(-(((xx - cx) ** 2 + (yy - cy) ** 2) / (2 * (r / 1.5) ** 2)))[..., None]
    return _clip(img * (1 - alpha) + 255 * alpha)


def occlusion(img, s, rng):
    h, w = img.shape[:2]
    area = _sev(s, [0.03, 0.06, 0.10]) * h * w
    aspect = rng.uniform(0.5, 2.0)
    ph, pw = int(np.sqrt(area / aspect)), int(np.sqrt(area * aspect))
    y0, x0 = rng.integers(0, h - ph), rng.integers(0, w - pw)
    out = img.copy()
    out[y0 : y0 + ph, x0 : x0 + pw] = np.median(img.reshape(-1, 3), 0)  # wall-colored patch
    return out


# Ordered as a physical pipeline: scene/geometry -> lighting -> optics -> sensor -> compression.
CORRUPTIONS = {
    "occlusion": occlusion,
    "glare": glare,
    "perspective": perspective,
    "rotate": rotate,
    "translate": translate,
    "zoom": zoom,
    "brightness": brightness,
    "uneven_light": uneven_light,
    "contrast": contrast,
    "white_balance": white_balance,
    "desaturate": desaturate,
    "defocus": defocus,
    "motion_blur": motion_blur,
    "low_res": low_res,
    "gaussian_noise": gaussian_noise,
    "jpeg": jpeg,
}
NAMES = list(CORRUPTIONS)


def composite(img, rng, k_range=(2, 4)):
    """2-4 random corruptions at random severities, applied in pipeline order."""
    k = int(rng.integers(k_range[0], k_range[1] + 1))
    chosen = sorted(rng.choice(len(NAMES), size=k, replace=False))
    applied = []
    for i in chosen:
        s = int(rng.integers(1, 4))
        img = CORRUPTIONS[NAMES[i]](img, s, rng)
        applied.append(f"{NAMES[i]}@{s}")
    return img, applied
