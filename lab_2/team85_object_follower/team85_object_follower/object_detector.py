# Authors: Chu-Rong Chen, Xingyu Zhu

from typing import Tuple, Optional

import cv2
import numpy as np
import platform
import math


# ============================================================
# Configuration
# ============================================================

# Process at a moderate resolution so HoughCircles can run
# fast enough for live video.
PROCESS_WIDTH = 640


# ============================================================
# Utility functions
# ============================================================


def nothing(x):
    pass


def resize_to_width(image, width):
    h, w = image.shape[:2]
    if w <= width:
        return image.copy(), 1.0

    scale = width / w
    resized = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    return resized, scale


def make_panel(image, title, width=400, height=300):
    """
    Make one panel for the debug dashboard.
    """

    if len(image.shape) == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    panel = cv2.resize(image, (width, height))

    # Dark title background
    cv2.rectangle(panel, (0, 0), (width, 35), (0, 0, 0), -1)
    cv2.putText(
        panel, title, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2
    )

    return panel


def create_debug_dashboard(debug):
    """
    Build a 2x3 debug view:

        Raw           HSV mask           RGB-ratio mask
        Combined      Edges              Detection
    """

    p1 = make_panel(debug["raw"], "1. Raw camera")
    p2 = make_panel(debug["hsv_mask"], "2. HSV mask")
    p3 = make_panel(debug["ratio_mask"], "3. RGB-ratio mask")
    p4 = make_panel(debug["combined_mask"], "4. Combined mask")
    p5 = make_panel(debug["edges"], "5. Edge image")
    p6 = make_panel(debug["result"], "6. Circle candidates / result")

    row1 = cv2.hconcat([p1, p2, p3])
    row2 = cv2.hconcat([p4, p5, p6])

    return cv2.vconcat([row1, row2])


# ============================================================
# Ball detector
# ============================================================


def detect_ball(
    frame,
    h_low,
    h_high,
    s_min,
    v_min,
    v_max,
    min_orange_ratio,
    hough_p2,
    canny_low,
    canny_high,
    process_width=PROCESS_WIDTH,
):
    """
    Detect a mostly-orange ball.

    Returns:
        detection:
            None, or dictionary containing:
                center
                radius
                orange_ratio
                edge_support

        debug:
            Images showing intermediate processing stages.
    """

    # --------------------------------------------------------
    # 1. Resize image for faster processing
    # --------------------------------------------------------

    image, scale = resize_to_width(frame, process_width)
    h, w = image.shape[:2]

    # --------------------------------------------------------
    # 2. Slight blur
    # --------------------------------------------------------

    blurred = cv2.GaussianBlur(image, (7, 7), 0)

    # --------------------------------------------------------
    # 3. Convert to HSV and extract orange pixels
    # --------------------------------------------------------

    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # The ball is red, whose hue sits right on the 0/179 seam of the
    # OpenCV hue circle (measured samples land on 177, 178, 0, 0).
    # When h_low > h_high the range is treated as wrapping around 0,
    # i.e. [h_low, 179] OR [0, h_high].
    if h_low > h_high:
        mask_hi = cv2.inRange(
            hsv,
            np.array([h_low, s_min, v_min]),
            np.array([179, 255, v_max]),
        )
        mask_lo = cv2.inRange(
            hsv,
            np.array([0, s_min, v_min]),
            np.array([h_high, 255, v_max]),
        )
        hsv_mask = cv2.bitwise_or(mask_hi, mask_lo)
    else:
        lower_color = np.array([h_low, s_min, v_min])
        upper_color = np.array([h_high, 255, v_max])
        hsv_mask = cv2.inRange(hsv, lower_color, upper_color)

    # --------------------------------------------------------
    # 4. RGB-ratio filtering
    #
    # Measured ball colors (four samples, lit and shadowed):
    #   RGB = (218, 100, 106)  R/G = 2.18  R/B = 2.06
    #   RGB = (140,  35,  36)  R/G = 4.00  R/B = 3.89
    #   RGB = (139,  39,  39)  R/G = 3.56  R/B = 3.56
    #   RGB = (214,  98, 109)  R/G = 2.18  R/B = 1.96
    #
    # This ball is red, not orange: G and B are roughly equal
    # (G/B = 0.90..1.00), so the old "g > 1.10 * b" orange test
    # rejected every real ball pixel. Instead require red to
    # dominate BOTH other channels, at ~0.85x the weakest
    # measured ratio so illumination changes still pass:
    #   R/G > 1.85  (weakest measured 2.18)
    #   R/B > 1.65  (weakest measured 1.96)
    #
    # Skin tone has a much flatter R/G (~1.2-1.5) and so is
    # still rejected here.
    # --------------------------------------------------------

    b, g, r = cv2.split(blurred)

    b = b.astype(np.float32)
    g = g.astype(np.float32)
    r = r.astype(np.float32)

    ratio_mask = ((r > 1.85 * g) & (r > 1.65 * b)).astype(np.uint8) * 255
    combined_mask = cv2.bitwise_and(hsv_mask, ratio_mask)

    # --------------------------------------------------------
    # 5. Morphological cleanup
    #
    # Small holes/noise are removed, but large white/black
    # patterns on the ball are intentionally NOT filled.
    # --------------------------------------------------------

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    clean_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    clean_mask = cv2.morphologyEx(clean_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    # --------------------------------------------------------
    # 6. Prepare grayscale image for circle detection
    # --------------------------------------------------------

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (9, 9), 2)

    # --------------------------------------------------------
    # 7. Edge image
    #
    # Used here for both visualization and scoring.
    #
    # NOTE: gray was blurred with an explicit sigma=2 (step 6),
    # not just a kernel size. Sigma sets the real blur strength;
    # the kernel size barely matters once it's big enough to
    # cover that sigma. That blur caps how strong an edge's
    # gradient can get: a step of brightness difference `diff`
    # peaks at roughly `1.5 * diff` after this blur (measured).
    # So canny_high must be well below 1.5x the smallest real
    # edge contrast you expect, or Canny finds zero seed pixels
    # and produces nothing regardless of image resolution.
    # (The old hardcoded (80, 160) required diff > ~105 just to
    # register at all -- too strict for most real lighting.)
    # --------------------------------------------------------

    edges = cv2.Canny(gray, canny_low, canny_high)

    # --------------------------------------------------------
    # 8. Hough Circle Transform
    #
    # Radius range is intentionally very broad.
    # This allows the ball to move toward/away from camera.
    # --------------------------------------------------------

    min_dimension = min(h, w)
    min_radius = max(4, int(min_dimension * 0.01))
    max_radius = int(min_dimension * 0.48)
    min_distance = max(20, int(min_dimension * 0.08))

    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=min_distance,
        # Canny high threshold used internally (kept equal to
        # canny_high so panel 5 always matches what Hough sees --
        # previously this was a hardcoded 120, independent of the
        # explicit Canny call above, so tuning what you saw in
        # panel 5 didn't actually change what Hough used).
        param1=canny_high,
        # Smaller = more sensitive, but more false circles
        param2=hough_p2,
        minRadius=min_radius,
        maxRadius=max_radius,
    )

    # --------------------------------------------------------
    # 9. Evaluate each circle using orange content
    # --------------------------------------------------------

    result = image.copy()

    best_candidate = None
    best_score = -1

    yy, xx = np.ogrid[:h, :w]
    if circles is not None:
        circles = circles[0]

        for x, y, radius in circles:
            # ----------------------------------------------
            # Pixels inside candidate circle
            # ----------------------------------------------
            distance_squared = (xx - x) ** 2 + (yy - y) ** 2
            inside_circle = distance_squared <= radius**2
            number_inside = np.count_nonzero(inside_circle)

            if number_inside == 0:
                continue

            orange_inside = np.count_nonzero(clean_mask[inside_circle])
            orange_ratio = orange_inside / number_inside

            # ----------------------------------------------
            # Measure edge support around circumference
            #
            # A real ball should have at least part of its
            # outer circumference visible.
            # ----------------------------------------------

            ring_thickness = max(2, int(radius * 0.015))
            inner_radius = max(1, radius - ring_thickness)
            outer_radius = radius + ring_thickness
            ring = (distance_squared >= inner_radius**2) & (
                distance_squared <= outer_radius**2
            )
            ring_pixels = np.count_nonzero(ring)

            if ring_pixels > 0:
                edge_pixels = np.count_nonzero(edges[ring])
                edge_support = edge_pixels / ring_pixels
            else:
                edge_support = 0

            # ----------------------------------------------
            # Reject circles that do not contain enough
            # orange.
            #
            # Notice that this does NOT require the entire
            # ball to be orange.
            # ----------------------------------------------

            if orange_ratio < min_orange_ratio:
                continue

            # ----------------------------------------------
            # Candidate score
            #
            # orange_ratio:
            #     should contain lots of orange
            #
            # sqrt(radius):
            #     suppresses small false circles inside
            #     the pattern without imposing a fixed size
            #
            # edge_support:
            #     rewards circles whose circumference
            #     corresponds to actual image edges
            # ----------------------------------------------

            score = orange_ratio * math.sqrt(radius) * (0.5 + 3.0 * edge_support)

            # Draw acceptable candidates in yellow
            cv2.circle(result, (int(x), int(y)), int(radius), (0, 255, 255), 1)

            if score > best_score:
                best_score = score
                best_candidate = {
                    "x": x,
                    "y": y,
                    "radius": radius,
                    "orange_ratio": orange_ratio,
                    "edge_support": edge_support,
                }

    # --------------------------------------------------------
    # 10. Draw best circle
    # --------------------------------------------------------

    detection = None

    if best_candidate is not None:
        x = best_candidate["x"]
        y = best_candidate["y"]
        radius = best_candidate["radius"]
        center = (int(x), int(y))

        # Green = chosen ball
        cv2.circle(result, center, int(radius), (0, 255, 0), 3)
        cv2.circle(result, center, 5, (0, 0, 255), -1)

        text1 = f"orange={best_candidate['orange_ratio']:.2f}"
        text2 = f"radius={radius:.1f}px"

        cv2.putText(
            result, text1, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )
        cv2.putText(
            result, text2, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )

        # Convert coordinates back to original camera resolution
        detection = {
            "center": (int(x / scale), int(y / scale)),
            "radius": radius / scale,
            "orange_ratio": best_candidate["orange_ratio"],
            "edge_support": best_candidate["edge_support"],
        }

    else:
        cv2.putText(
            result,
            "Ball not detected",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
        )

    debug = {
        "raw": image,
        "hsv_mask": hsv_mask,
        "ratio_mask": ratio_mask,
        "combined_mask": clean_mask,
        "edges": edges,
        "result": result,
    }

    return detection, debug


# ============================================================
# Main
# ============================================================


class ObjectDetector:
    """Ball detector that can run with or without OpenCV GUI windows."""

    def __init__(
        self,
        process_width: int = PROCESS_WIDTH,
        headless: bool = False,
        h_low: int = 172,
        h_high: int = 6,
        s_min: int = 110,
        v_min: int = 90,
        v_max: int = 255,
        min_orange_ratio: float = 0.30,
        hough_p2: int = 32,
        canny_low: int = 30,
        canny_high: int = 90,
    ):
        self.process_width = process_width
        self.headless = headless

        # Used directly in headless mode and as GUI trackbar defaults.
        self.h_low = h_low
        self.h_high = h_high
        self.s_min = s_min
        self.v_min = v_min
        self.v_max = v_max
        self.min_orange_ratio = min_orange_ratio
        self.hough_p2 = max(hough_p2, 1)
        self.canny_low = max(canny_low, 1)
        self.canny_high = max(canny_high, self.canny_low + 1)

        self.controls_window = "Detector Controls"
        self.total_frames = 0
        self.detected_frames = 0

    def create(self) -> None:
        self.reset_stats()
        if self.headless:
            return

        cv2.namedWindow(self.controls_window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.controls_window, 600, 340)
        cv2.createTrackbar("H low", self.controls_window, self.h_low, 179, nothing)
        cv2.createTrackbar("H high", self.controls_window, self.h_high, 179, nothing)
        cv2.createTrackbar("S min", self.controls_window, self.s_min, 255, nothing)
        cv2.createTrackbar("V min", self.controls_window, self.v_min, 255, nothing)
        cv2.createTrackbar("V max", self.controls_window, self.v_max, 255, nothing)
        cv2.createTrackbar(
            "Min orange %",
            self.controls_window,
            round(self.min_orange_ratio * 100),
            100,
            nothing,
        )
        cv2.createTrackbar("Hough p2", self.controls_window, self.hough_p2, 80, nothing)
        cv2.createTrackbar("Canny low", self.controls_window, self.canny_low, 300, nothing)
        cv2.createTrackbar("Canny high", self.controls_window, self.canny_high, 300, nothing)

        print()
        print("Controls:")
        print("  q or ESC : quit")
        print("  r        : reset detection-rate counter")
        print()

    def reset_stats(self) -> None:
        self.total_frames = 0
        self.detected_frames = 0

    def destroy(self) -> None:
        if not self.headless:
            cv2.destroyAllWindows()

    def _parameters(self):
        if self.headless:
            return (
                self.h_low,
                self.h_high,
                self.s_min,
                self.v_min,
                self.v_max,
                self.min_orange_ratio,
                self.hough_p2,
                self.canny_low,
                self.canny_high,
            )

        h_low = cv2.getTrackbarPos("H low", self.controls_window)
        h_high = cv2.getTrackbarPos("H high", self.controls_window)
        s_min = cv2.getTrackbarPos("S min", self.controls_window)
        v_min = cv2.getTrackbarPos("V min", self.controls_window)
        v_max = cv2.getTrackbarPos("V max", self.controls_window)
        min_orange_ratio = (
            cv2.getTrackbarPos("Min orange %", self.controls_window) / 100.0
        )
        hough_p2 = max(cv2.getTrackbarPos("Hough p2", self.controls_window), 1)
        canny_low = max(cv2.getTrackbarPos("Canny low", self.controls_window), 1)
        canny_high = max(
            cv2.getTrackbarPos("Canny high", self.controls_window), canny_low + 1
        )
        return (
            h_low,
            h_high,
            s_min,
            v_min,
            v_max,
            min_orange_ratio,
            hough_p2,
            canny_low,
            canny_high,
        )

    def detect(self, frame: np.ndarray) -> Tuple[int, Optional[Tuple[int, int]]]:
        if frame is None or not isinstance(frame, np.ndarray) or frame.size == 0:
            raise ValueError("frame must be a non-empty NumPy array")

        parameters = self._parameters()
        detection, debug = detect_ball(
            frame, *parameters, process_width=self.process_width
        )

        self.total_frames += 1
        if detection is not None:
            self.detected_frames += 1

        if not self.headless:
            detection_rate = 100.0 * self.detected_frames / self.total_frames
            dashboard = create_debug_dashboard(debug)
            cv2.imshow("Ball Detector Debug", dashboard)

            control_display = np.zeros((140, 600, 3), dtype=np.uint8)
            cv2.putText(
                control_display,
                "Tune HSV while watching stages 2 and 4.",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                1,
            )
            cv2.putText(
                control_display,
                "Lower Hough p2 = more sensitive.",
                (10, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                1,
            )
            detection_color = (0, 255, 0) if detection is not None else (0, 165, 255)
            cv2.putText(
                control_display,
                f"Detection rate: {detection_rate:5.1f}% "
                f"({self.detected_frames}/{self.total_frames} frames, 'r' to reset)",
                (10, 100),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                detection_color,
                1,
            )
            cv2.imshow(self.controls_window, control_display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                return 1, None
            if key == ord("r"):
                self.reset_stats()

        if detection is not None:
            return 0, detection["center"]
        return 0, None
