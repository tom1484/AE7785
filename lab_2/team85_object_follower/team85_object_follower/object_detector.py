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
    def __init__(self, process_width: int = PROCESS_WIDTH):
        self.process_width = process_width

    def create(self):
        # self.cap = open_camera(self.camera_id)

        # # Ask for decent camera resolution.
        # # The camera may choose a different resolution.
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # ========================================================
        # Live tuning controls
        # ========================================================

        self.controls_window = "Detector Controls"
        cv2.namedWindow(self.controls_window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.controls_window, 600, 340)

        # Ball samples (current camera):
        #   RGB = (218, 100, 106) -> HSV = (178, 138, 218)
        #   RGB = (140,  35,  36) -> HSV = (  0, 191, 140)
        #   RGB = (139,  39,  39) -> HSV = (  0, 183, 139)
        #   RGB = (214,  98, 109) -> HSV = (177, 138, 214)
        #
        # Hue straddles the 0/179 seam, so "H low" > "H high"
        # here: detect_ball reads that as the wrapping range
        # [172, 179] OR [0, 6].
        #
        # S spans 138..191 and V spans 139..218; the bounds below
        # sit outside both with margin, while S min stays high
        # enough to reject skin.
        cv2.createTrackbar("H low", self.controls_window, 172, 179, nothing)
        cv2.createTrackbar("H high", self.controls_window, 6, 179, nothing)
        cv2.createTrackbar("S min", self.controls_window, 110, 255, nothing)
        cv2.createTrackbar("V min", self.controls_window, 90, 255, nothing)
        cv2.createTrackbar("V max", self.controls_window, 255, 255, nothing)

        # 30 = circle only needs 30% orange
        #
        # Your patterned ball should normally have much more
        # orange than this, but this tolerates the white pattern,
        # shadows, and partial occlusion.
        cv2.createTrackbar("Min orange %", self.controls_window, 30, 100, nothing)

        # Lower value:
        #     detects more circles
        #     more false positives
        #
        # Higher value:
        #     stricter circle detection
        cv2.createTrackbar("Hough p2", self.controls_window, 32, 80, nothing)

        # Canny thresholds for the edge image (panel 5) and for
        # Hough's internal edge detection (see detect_ball step 8 --
        # param1 is kept equal to "Canny high" so what you see in
        # panel 5 is what Hough actually uses).
        #
        # gray is blurred with sigma=2 before this (step 6), which
        # caps how strong an edge's gradient can get: a brightness
        # step of size `diff` peaks at roughly `1.5 * diff` after
        # that blur. So canny_high must stay below ~1.5x your
        # smallest real edge contrast, or Canny finds no seed
        # pixels and panel 5 goes black no matter how sharp the
        # physical edge looks. Sample a lit patch and a shadowed/
        # background patch next to a real edge, take their gray
        # difference, and keep "Canny high" comfortably under 1.5x
        # that number.
        cv2.createTrackbar("Canny low", self.controls_window, 30, 300, nothing)
        cv2.createTrackbar("Canny high", self.controls_window, 90, 300, nothing)

        # ========================================================
        # Detection-rate counter
        #
        # Tracks what fraction of processed frames produced a
        # detection, so tuning changes can be judged quantitatively
        # instead of just by eye.
        # ========================================================

        self.total_frames = 0
        self.detected_frames = 0

        print()
        print("Controls:")
        print("  q or ESC : quit")
        print("  r        : reset detection-rate counter")
        print()
        print("If the Raw Camera panel is black,")
        print("the problem is camera input, not image processing.")
        print()

    def reset_stats(self):
        self.total_frames = 0
        self.detected_frames = 0

    def destroy(self):
        cv2.destroyAllWindows()

    def detect(self, frame: np.ndarray) -> Tuple[int, Optional[Tuple[int, int]]]:
        try:
            # ----------------------------------------------------
            # Read live parameters
            # ----------------------------------------------------

            h_low = cv2.getTrackbarPos("H low", self.controls_window)
            h_high = cv2.getTrackbarPos("H high", self.controls_window)
            s_min = cv2.getTrackbarPos("S min", self.controls_window)
            v_min = cv2.getTrackbarPos("V min", self.controls_window)
            v_max = cv2.getTrackbarPos("V max", self.controls_window)

            min_orange_ratio = (
                cv2.getTrackbarPos("Min orange %", self.controls_window) / 100.0
            )

            hough_p2 = cv2.getTrackbarPos("Hough p2", self.controls_window)

            # Prevent invalid Hough threshold
            hough_p2 = max(hough_p2, 1)

            canny_low = cv2.getTrackbarPos("Canny low", self.controls_window)
            canny_high = cv2.getTrackbarPos("Canny high", self.controls_window)

            # Prevent invalid/degenerate Canny thresholds
            canny_low = max(canny_low, 1)
            canny_high = max(canny_high, canny_low + 1)

            # ----------------------------------------------------
            # Detect
            # ----------------------------------------------------

            detection, debug = detect_ball(
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
                process_width=self.process_width,
            )

            # ----------------------------------------------------
            # Update detection-rate counter
            # ----------------------------------------------------

            self.total_frames += 1
            if detection is not None:
                self.detected_frames += 1
            detection_rate = (
                100.0 * self.detected_frames / self.total_frames
                if self.total_frames > 0
                else 0.0
            )

            # ----------------------------------------------------
            # Print ball position
            # ----------------------------------------------------

            if detection is not None:
                x, y = detection["center"]
                print(
                    f"\rBall: "
                    f"x={x:4d}, "
                    f"y={y:4d}, "
                    f"r={detection['radius']:6.1f}, "
                    f"orange={detection['orange_ratio']:.2f}, "
                    f"detected={detection_rate:5.1f}% ({self.detected_frames}/{self.total_frames})",
                    end="",
                )
            else:
                print(
                    f"\rBall: not detected, "
                    f"detected={detection_rate:5.1f}% ({self.detected_frames}/{self.total_frames})",
                    end="",
                )

            # ----------------------------------------------------
            # Debug dashboard
            # ----------------------------------------------------

            dashboard = create_debug_dashboard(debug)
            cv2.imshow("Ball Detector Debug", dashboard)

            # Keep control window visible
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

        except Exception as e:
            raise e

        if detection is not None:
            return 0, detection["center"]
        else:
            return 0, None
