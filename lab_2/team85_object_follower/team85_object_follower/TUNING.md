# Tuning the ball detector

`object_detector.py` finds the ball by combining an HSV color
mask, an RGB-ratio mask, and a Hough circle transform, then scoring
each candidate circle. This guide explains what each control does and
how to tune it, using the debug dashboard and the on-screen
**detection rate** counter (`detected_frames / total_frames`, shown in
the "Detector Controls" window and printed to the terminal each
frame) as your feedback signal.

Press `r` at any time to reset the counter — do this whenever you
change a parameter so the percentage reflects only the current
setting, not frames captured under the old one.

## General workflow

1. Point the camera at the ball under the lighting/background you
   actually care about (or several, if the environment varies).
2. Reset the counter (`r`).
3. Watch panel **2 (HSV mask)** and panel **4 (Combined mask)** while
   nudging one trackbar at a time.
4. Let it run for a while (tens to hundreds of frames, including some
   motion and different distances), then read the detection rate.
5. Compare rates across parameter choices instead of eyeballing a
   single frame — a mask can look "clean" on one frame and still miss
   most others.
6. Also check for **false positives** (a circle drawn when there is
   no ball, or locked onto the wrong object) — the detection rate
   alone can't tell you that, so glance at panel **6** too.

Tune in this order: HSV bounds → RGB ratio → morphology → Canny edge
thresholds → Hough sensitivity → orange-ratio threshold. Each stage
depends on the one before it, so re-tuning HSV after you've dialed in
Hough usually undoes your Hough tuning.

## 1. Color bounds (HSV mask, panel 2)

Trackbars: `H low`, `H high`, `S min`, `V min`, `V max`.

The HSV mask (`cv2.inRange`) keeps pixels whose hue is in
`[H low, H high]`, saturation is at least `S min`, and value (brightness)
is in `[V min, V max]`.

**How to find the right bounds for your ball:**

- Sample the ball's actual color instead of guessing. With the camera
  pointed at the ball, crop a small patch and check its average BGR
  value (e.g. in a Python/OpenCV one-liner, or a paint program's color
  picker on a saved frame). Convert that BGR to HSV
  (`cv2.cvtColor(np.uint8([[[b, g, r]]]), cv2.COLOR_BGR2HSV)`) to get a
  center hue/sat/val to tune around.
- **Hue (`H low`/`H high`):** OpenCV hue runs 0–179. The current ball
  is **red**, whose hue straddles the 0/179 seam — the measured
  samples land on 177, 178, 0 and 0. `detect_ball` therefore supports
  a **wrapping** hue range: when `H low > H high` it builds two masks,
  `[H low, 179]` and `[0, H high]`, and ORs them together. The
  defaults are `H low = 172`, `H high = 6`, which is that wrapping
  range, and both trackbars are capped at 179 (the real hue maximum)
  rather than 255.
  - Keep `H low > H high` while the ball is red. If you set
    `H low < H high` the code falls back to a single plain range, and
    any pixel on the far side of the seam is silently dropped — this
    is exactly the bug the old `150–200` default had, which passed
    hues 150–179 but rejected the samples sitting at hue 0.
  - To widen the range, move the two ends *outward from the seam*:
    lower `H low` (172 → 168) to take in more magenta-leaning red,
    raise `H high` (6 → 10) to take in more orange-leaning red.
- **Saturation (`S min`):** Skin, wood, and other tan/orange-ish
  backgrounds usually have *lower* saturation than a saturated plastic
  or foam ball. Raising `S min` is the single most effective way to
  reject skin tones — watch panel 2 while raising it until skin
  disappears but the ball doesn't fade.
- **Value (`V min`/`V max`):** These bound brightness, not color.
  `V min` rejects dark shadowed regions (including the ball's own
  shadowed side); `V max` rejects blown-out/glare regions. If the mask
  flickers on/off as the ball moves through mixed lighting, widen
  these first before touching hue/saturation.
- **Symptom → fix:**
  - Mask is empty even centered on the ball → hue range too narrow, or
    `S min`/`V min` too high. Widen incrementally.
  - Mask catches only *part* of the ball, and the missing part is a
    consistent shade → you have probably lost the hue wraparound;
    check that `H low` is still greater than `H high`.
  - Mask includes background/skin → narrow the hue range, or raise
    `S min`.
  - Mask flickers under normal lighting → widen `V min`/`V max`.

## 2. RGB-ratio mask (panel 3)

This is a second, independent color filter (`detect_ball`, ratio_mask
block) that isn't exposed as a trackbar — it's hardcoded from measured
sample colors:

| Sample RGB | R/G | G/B | R/B |
|---|---|---|---|
| (218, 100, 106) | 2.18 | 0.94 | 2.06 |
| (140, 35, 36) | 4.00 | 0.97 | 3.89 |
| (139, 39, 39) | 3.56 | 1.00 | 3.56 |
| (214, 98, 109) | 2.18 | 0.90 | 1.96 |

then loosened to ~0.85× the weakest measured ratio:

```python
ratio_mask = (r > 1.85 * g) & (r > 1.65 * b)
```

Its job is to reject things HSV alone can't — in particular skin tone,
which can land inside a loose red/orange HSV range but has a much
flatter R/G ratio (~1.2–1.5) than a saturated ball.

**Why this test is `R > G` and `R > B`, not the old `G > B`:** the
previous version was written for an *orange* ball and required
`g > 1.10 * b`, i.e. green clearly above blue. On this red ball G and
B are near-equal (G/B = 0.90–1.00), so that test rejected **every**
real ball pixel and panel 4 was always black no matter how the HSV
trackbars were set. The rule now just asks that red dominate both
other channels, which is what distinguishes red from grey, skin and
wood alike.

**If you re-tune this for a different-colored ball or camera:**

1. Sample several patches of the ball (different lighting angles) and
   compute the average `R/G` and `G/B` ratios.
2. Set the two thresholds *below* your measured ratios (e.g. ~0.85–0.9×
   the measured value) so real ball pixels reliably pass while still
   excluding flatter-ratio backgrounds.
3. Since these aren't trackbars, edit the constants in
   `object_detector.py` and re-run — use panel 3 and the detection-rate
   counter the same way you did for HSV.
4. If your target isn't red/orange at all, this specific ratio test
   (`R` dominant over both other channels) won't apply — replace it with
   whatever ratio relationship distinguishes your target's color from
   the background, or drop this stage entirely and rely on HSV alone.

If panel 4 (combined mask) is much emptier than panel 2 (HSV mask),
the ratio mask is the bottleneck — either it's miscalibrated for your
lighting, or your background genuinely has similar color ratios to
the target and this filter won't help.

## 3. Morphological cleanup

Not exposed as trackbars, but worth knowing about when tuning masks:

```python
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
clean_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel, iterations=1)
clean_mask = cv2.morphologyEx(clean_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
```

- `MORPH_OPEN` erodes-then-dilates to remove small speckle noise
  (isolated single pixels/small blobs from the color masks).
- `MORPH_CLOSE` dilates-then-erodes to fill small gaps/holes (e.g. the
  ball's white pattern shouldn't create holes that lower its orange
  ratio too much).
- If real ball area is being eaten away, shrink the kernel (e.g. to
  `(3, 3)`) or drop an iteration. If background speckle is still
  getting through to panel 4, grow the kernel or add an iteration.
- This step is a tradeoff against the min-orange-ratio threshold
  (below): more aggressive closing means a patterned/partially
  occluded ball keeps a higher orange ratio, letting you raise the
  min-ratio threshold and reject more false positives elsewhere.

## 4. Canny edge thresholds (`Canny low`/`Canny high`, panel 5)

`edges = cv2.Canny(gray, canny_low, canny_high)` — run on the
grayscale image (independent of the color masks). `canny_high` is
also fed straight into Hough as `param1` (its own internal edge
detector), so panel 5 always shows exactly what Hough sees; the two
used to be different hardcoded numbers (explicit `Canny(80, 160)` for
the panel vs. Hough's own `param1=120`), so tuning what you *saw*
didn't actually change what Hough used.

**Why edges go missing regardless of resolution or ball color:**
`gray` is blurred with an *explicit* `sigma=2` right before this
(`detect_ball`, step 6) — `cv2.GaussianBlur(gray, (9, 9), 2)`. Sigma
sets the real blur strength; the `(9, 9)` kernel size barely matters
once it's large enough to contain that sigma, so shrinking the kernel
does ~nothing. That blur caps the gradient a real edge can produce: a
step of brightness difference `diff` peaks at only about `1.5 * diff`
afterward (measured empirically). Concretely:

| Brightness diff across the edge | Peak gradient after blur | Passes old `high=160`? |
|---|---|---|
| 40 | 60 | no |
| 60 | 92 | no |
| 80 | 124 | no |
| 100 | 152 | no |
| 120 | 184 | yes |

The old hardcoded thresholds (`80, 160`, Hough `param1=120`) needed a
brightness difference over the edge of roughly **105** just to
register a single seed pixel — most real indoor lighting doesn't
produce contrast that stark, so panel 5 (and Hough) would go dark even
with a sharp, well-focused, well-color-masked ball. This has nothing
to do with the 320×240 frame size; the same math applies at any
resolution.

**How to tune it:**
1. Sample gray values on both sides of a real edge of the ball (e.g.
   ball vs. background right at its rim) from a saved frame.
2. Take their difference, multiply by ~1.5 — that's roughly the peak
   gradient Canny will actually see for that edge.
3. Set `Canny high` comfortably *below* that number so the edge
   registers as a seed pixel; set `Canny low` at roughly a third to a
   half of `Canny high` so hysteresis can extend along weaker parts of
   the same edge.
4. Watch panel 5 while nudging — you want the ball's outline traced,
   not a solid white panel (background noise/texture also passing) and
   not a blank one (thresholds still too high).
5. If you lower both a lot and start picking up background texture as
   edges, raise `Canny low` first (it controls how much of that noise
   survives hysteresis) before raising `Canny high` back up.

A background whose gray level is genuinely identical to the ball's
(true isoluminance, not just "old thresholds too strict") can't be
fixed by this knob at all — panel 5 will show 0 edges there no matter
how low you go, because there is no gradient to find. That's a sign to
change the lighting/background, not the thresholds.

## 5. Hough circle sensitivity (`Hough p2`, panel 6)

`cv2.HoughCircles(..., param2=hough_p2, ...)` — this is the
accumulator threshold for the circle transform, run on the same
Canny edges from step 4 (independent of the color masks).

- **Lower `Hough p2`** → more candidate circles proposed (higher
  recall, more false positives to filter downstream).
- **Higher `Hough p2`** → fewer, stricter candidates (higher
  precision, may miss the ball if lighting/edges are weak).

Circle candidates are drawn in **yellow** on panel 6 if they pass the
orange-ratio filter; the winning circle is drawn in **green**. If you
see no yellow circles near the ball at all, first check panel 5 has a
visible outline (step 4) — nothing downstream can fix a circle that
Hough never proposed because it saw no edges. If panel 5 looks right
and you still see nothing, then lower `Hough p2`. If you see many
yellow circles scattered over background clutter, raise it (and/or
improve the color masks so more of them get rejected by the
orange-ratio filter).

Radius search range (`minRadius`/`maxRadius`) and `minDist` between
circles are computed automatically from frame size
(`detect_ball`, step 8) rather than exposed as trackbars — if you know
the ball only ever appears within a narrower size range at your
working distances, tightening these in code will reduce false
positives and speed up Hough.

**Radius flickers between two values even though panel 5 looks
stable.** This means the problem isn't the color masks or Canny (both
upstream of Hough) — it's the ball itself. A marking, highlight,
shadow, or seam on the ball's surface is a second, *genuinely real*,
smaller circle concentric with the true outer rim. Reproduced this
directly on a synthetic ball with an inner marking: across 10 noisy
frames, total edge-pixel count barely moved (511–519, ~1% variation —
invisible by eye in panel 5), yet the winning radius alternated
between ~49px (the true rim) and 22px (the inner marking) on exactly
one frame out of ten.

The mechanism: on that one frame, ordinary per-pixel noise pushed the
outer rim's Hough accumulator vote just under `Hough p2`, so Hough
didn't even *propose* the outer circle that frame — only the inner
one. Scoring never got a chance to compare them, since it only ranks
whatever Hough actually proposed. This is why **raising `Hough p2`
does not fix this failure mode** — it can only make the true circle
drop out more often, never less; you'd be tightening the exact
threshold that's already failing intermittently.

The fix that actually worked in testing: narrow `minRadius` (the code
constant in `detect_ball`, step 8) above the inner marking's size, so
it's excluded from consideration by size regardless of whether Hough's
vote for it happens to be strong that frame. In the reproduction,
`minRadius=2` flickered on 1 of 10 frames; raising it past the inner
circle's radius (`minRadius=35` against outer rim 50 / inner marking
22) locked onto the true rim on all 10. This only works if you know
the ball's on-screen size never legitimately drops that low at your
working distances — if it does (ball moving far away), you'll need to
suppress the false candidate a different way, e.g. checking whether
`Min orange %` (step 6) separates the marking's color from the ball's
true color.

If instead panel 6 shows a single yellow circle whose radius itself
wobbles by only a few pixels (not a full snap between two very
different values), that's ordinary Hough accumulator quantization
noise rather than a competing real circle — narrowing `minRadius`
won't help there; that's better addressed with temporal smoothing
across frames (not something a single-frame trackbar can fix, since
`HoughCircles` has no memory between calls).

## 6. Minimum orange ratio (`Min orange %`)

After Hough proposes circles, each one is scored using
`orange_ratio` = fraction of pixels inside the circle that are in
`clean_mask` (panel 4). Circles below `Min orange %` are discarded
outright, before scoring.

- Raise this to reject circles that only clip the ball's edge or sit
  mostly over background.
- Lower this if your ball has large non-orange markings (patterns,
  logos, glare) that legitimately reduce its own orange coverage —
  the comment in the code notes the default (30%) is deliberately
  loose to tolerate a patterned ball, shadows, and partial occlusion.
- If you raise this and the detection rate drops without panel 6
  showing obviously-wrong circles disappearing, your color mask
  coverage of the true ball (panels 2–4) is probably the real
  limiter — go back to step 1.

## 7. Scoring (not a trackbar, but affects which circle wins)

When multiple circles pass the orange-ratio filter, the winner is
chosen by:

```python
score = orange_ratio * sqrt(radius) * (0.5 + 3.0 * edge_support)
```

`edge_support` is the fraction of the circle's circumference that
lines up with a real Canny edge (panel 5) — this favors circles whose
outline is actually visible in the image, not just circles that
happen to sit over an orange blob. If the detector keeps locking onto
a small spurious circle inside a larger true one, this is the
mechanism to look at (in code): weighting `edge_support` more heavily
should help, at the cost of missing balls with weak/blurry edges
(motion blur, low contrast against background).

## Quick reference: symptom → where to look

| Symptom | Likely cause | Where to tune |
|---|---|---|
| Detection rate near 0%, panel 2 mostly black | HSV bounds too narrow | Step 1 |
| Panel 2 has color, panel 4 mostly black | Ratio mask rejecting real ball pixels | Step 2 |
| Panels 2–4 all look good, panel 5 nearly/entirely black | `Canny high`/`low` too strict for your real edge contrast (very likely, was true even at the old hardcoded 80/160) | Step 4 |
| Panels 2–5 all look good, no yellow circles in panel 6 | `Hough p2` too strict | Step 5 |
| Panel 4 always black however you set HSV | RGB-ratio mask miscalibrated for the ball's color | Step 2 |
| Only part of the ball survives in panel 2 | Hue wraparound lost (`H low` must stay > `H high`) | Step 1 |
| Yellow circles on background, wrong circle turns green | Orange-ratio threshold too low, or masks leaking background | Steps 1–2, 6 |
| Detection flickers between adjacent frames | Borderline `V min`/`V max`, `Canny`, or `Hough p2` right at threshold | Step 1, Step 4, Step 5 |
| Radius snaps between two very different values, panel 5 looks stable | Ball's own marking/highlight is a real second circle; Hough's vote for the true rim occasionally dips below `Hough p2` | Step 5, narrow `minRadius` in code (raising `Hough p2` won't fix this) |
| Works at one distance, fails at another | Radius range too narrow, or lighting changes with distance | Step 5 radius bounds (in code) |
