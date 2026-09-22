# Tuning the ball detector

`find_object.py` finds a mostly-orange ball by combining an HSV color
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

Tune in this order: HSV bounds → RGB ratio → morphology → Hough
sensitivity → orange-ratio threshold. Each stage depends on the one
before it, so re-tuning HSV after you've dialed in Hough usually
undoes your Hough tuning.

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
- **Hue (`H low`/`H high`):** OpenCV hue runs 0–179. Red/orange sits
  near hue 0, and wraps around to near 179. If your ball's hue is
  genuinely near 0 (as in the current default, `0–12`), you only need
  the low end. If tuning reveals the ball's hue is closer to
  ~170–179 (a more red-leaning orange), you will need to handle the
  wraparound — either shift the bounds up, or build two masks (one for
  `[170, 179]` and one for `[0, X]`) and OR them together.
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
  - Mask includes background/skin → narrow the hue range, or raise
    `S min`.
  - Mask flickers under normal lighting → widen `V min`/`V max`.

## 2. RGB-ratio mask (panel 3)

This is a second, independent color filter (`detect_ball`, ratio_mask
block) that isn't exposed as a trackbar — it's hardcoded from a
measured sample color (`RGB ≈ (137, 55, 37)` → `R/G ≈ 2.49`,
`G/B ≈ 1.49`), then loosened to:

```python
ratio_mask = (r > 1.65 * g) & (g > 1.10 * b)
```

Its job is to reject things HSV alone can't — in particular skin tone,
which can land inside a loose orange HSV range but has a flatter
R/G/B ratio than a saturated orange ball.

**If you re-tune this for a different-colored ball or camera:**

1. Sample several patches of the ball (different lighting angles) and
   compute the average `R/G` and `G/B` ratios.
2. Set the two thresholds *below* your measured ratios (e.g. ~0.85–0.9×
   the measured value) so real ball pixels reliably pass while still
   excluding flatter-ratio backgrounds.
3. Since these aren't trackbars, edit the constants in
   `find_object.py` and re-run — use panel 3 and the detection-rate
   counter the same way you did for HSV.
4. If your target isn't orange/red at all, this specific ratio test
   (`R` dominant, then `G` over `B`) won't apply — replace it with
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

## 4. Hough circle sensitivity (`Hough p2`, panel 6)

`cv2.HoughCircles(..., param2=hough_p2, ...)` — this is the
accumulator threshold for the circle transform, run on the grayscale
image (independent of the color masks).

- **Lower `Hough p2`** → more candidate circles proposed (higher
  recall, more false positives to filter downstream).
- **Higher `Hough p2`** → fewer, stricter candidates (higher
  precision, may miss the ball if lighting/edges are weak).

Circle candidates are drawn in **yellow** on panel 6 if they pass the
orange-ratio filter; the winning circle is drawn in **green**. If you
see no yellow circles near the ball at all, lower `Hough p2` first —
nothing downstream can fix a circle that Hough never proposed. If you
see many yellow circles scattered over background clutter, raise it
(and/or improve the color masks so more of them get rejected by the
orange-ratio filter).

Radius search range (`minRadius`/`maxRadius`) and `minDist` between
circles are computed automatically from frame size
(`detect_ball`, step 8) rather than exposed as trackbars — if you know
the ball only ever appears within a narrower size range at your
working distances, tightening these in code will reduce false
positives and speed up Hough.

## 5. Minimum orange ratio (`Min orange %`)

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

## 6. Scoring (not a trackbar, but affects which circle wins)

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
| Panel 4 looks good, no yellow circles in panel 6 | Hough too strict, or edges too weak | Step 4, check Canny thresholds |
| Yellow circles on background, wrong circle turns green | Orange-ratio threshold too low, or masks leaking background | Steps 1–2, 5 |
| Detection flickers between adjacent frames | Borderline `V min`/`V max`, or `Hough p2` right at threshold | Step 1, Step 4 |
| Works at one distance, fails at another | Radius range too narrow, or lighting changes with distance | Step 4 radius bounds (in code) |
