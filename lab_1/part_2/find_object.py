# Authors: Chu-Rong Chen, Xingyu Zhu

import cv2
import numpy as np
import platform
import math


# ============================================================
# Configuration
# ============================================================

CAMERA_ID = 1

# Process at a moderate resolution so HoughCircles can run
# fast enough for live video.
PROCESS_WIDTH = 640


# ============================================================
# Camera
# ============================================================


def open_camera(camera_id):
    """
    Open the camera.

    On macOS, explicitly try AVFoundation first.
    """

    if platform.system() == "Darwin":
        print("Trying macOS AVFoundation camera backend...")

        cap = cv2.VideoCapture(camera_id, cv2.CAP_AVFOUNDATION)

        if cap.isOpened():
            return cap

        print("AVFoundation failed. Trying default backend...")

    cap = cv2.VideoCapture(camera_id)

    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera {camera_id}")

    return cap


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


def detect_ball(frame, h_low, h_high, s_min, v_min, v_max, min_orange_ratio, hough_p2):
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

    image, scale = resize_to_width(frame, PROCESS_WIDTH)

    h, w = image.shape[:2]

    # --------------------------------------------------------
    # 2. Slight blur
    # --------------------------------------------------------

    blurred = cv2.GaussianBlur(image, (7, 7), 0)

    # --------------------------------------------------------
    # 3. Convert to HSV and extract orange pixels
    # --------------------------------------------------------

    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    lower_orange = np.array([h_low, s_min, v_min])

    upper_orange = np.array([h_high, 255, v_max])

    hsv_mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # --------------------------------------------------------
    # 4. RGB-ratio filtering
    #
    # Measured ball color:
    #   RGB ~= (137, 55, 37)
    #
    # Therefore:
    #   R/G ~= 2.49
    #   G/B ~= 1.49
    #
    # Use looser thresholds to tolerate illumination changes
    # while rejecting much of the skin-colored background.
    # --------------------------------------------------------

    b, g, r = cv2.split(blurred)

    b = b.astype(np.float32)
    g = g.astype(np.float32)
    r = r.astype(np.float32)

    ratio_mask = ((r > 1.65 * g) & (g > 1.10 * b)).astype(np.uint8) * 255

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
    # --------------------------------------------------------

    edges = cv2.Canny(gray, 80, 160)

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
        # Canny high threshold used internally
        param1=120,
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


def main():

    cap = open_camera(CAMERA_ID)

    # Ask for decent camera resolution.
    # The camera may choose a different resolution.
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)

    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # ========================================================
    # Live tuning controls
    # ========================================================

    controls_window = "Detector Controls"

    cv2.namedWindow(controls_window, cv2.WINDOW_NORMAL)

    cv2.resizeWindow(controls_window, 600, 340)

    # Ball sample:
    #   RGB = (137, 55, 37)
    #   OpenCV HSV ~= (5, 186, 137)
    #
    # Use a relatively narrow hue range and high minimum
    # saturation to reduce skin detection.
    cv2.createTrackbar("H low", controls_window, 0, 179, nothing)

    cv2.createTrackbar("H high", controls_window, 12, 179, nothing)

    cv2.createTrackbar("S min", controls_window, 145, 255, nothing)

    cv2.createTrackbar("V min", controls_window, 60, 255, nothing)

    cv2.createTrackbar("V max", controls_window, 210, 255, nothing)

    # 30 = circle only needs 30% orange
    #
    # Your patterned ball should normally have much more
    # orange than this, but this tolerates the white pattern,
    # shadows, and partial occlusion.
    cv2.createTrackbar("Min orange %", controls_window, 30, 100, nothing)

    # Lower value:
    #     detects more circles
    #     more false positives
    #
    # Higher value:
    #     stricter circle detection
    cv2.createTrackbar("Hough p2", controls_window, 32, 80, nothing)

    # ========================================================
    # Detection-rate counter
    #
    # Tracks what fraction of processed frames produced a
    # detection, so tuning changes can be judged quantitatively
    # instead of just by eye.
    # ========================================================

    total_frames = 0
    detected_frames = 0

    print()
    print("Controls:")
    print("  q or ESC : quit")
    print("  r        : reset detection-rate counter")
    print()
    print("If the Raw Camera panel is black,")
    print("the problem is camera input, not image processing.")
    print()

    while True:

        ret, frame = cap.read()

        if not ret or frame is None:
            print("Failed to read camera frame.")
            break

        # ----------------------------------------------------
        # Read live parameters
        # ----------------------------------------------------

        h_low = cv2.getTrackbarPos("H low", controls_window)

        h_high = cv2.getTrackbarPos("H high", controls_window)

        s_min = cv2.getTrackbarPos("S min", controls_window)

        v_min = cv2.getTrackbarPos("V min", controls_window)

        v_max = cv2.getTrackbarPos("V max", controls_window)

        min_orange_ratio = cv2.getTrackbarPos("Min orange %", controls_window) / 100.0

        hough_p2 = cv2.getTrackbarPos("Hough p2", controls_window)

        # Prevent invalid Hough threshold
        hough_p2 = max(hough_p2, 1)

        # ----------------------------------------------------
        # Detect
        # ----------------------------------------------------

        detection, debug = detect_ball(
            frame, h_low, h_high, s_min, v_min, v_max, min_orange_ratio, hough_p2
        )

        # ----------------------------------------------------
        # Update detection-rate counter
        # ----------------------------------------------------

        total_frames += 1

        if detection is not None:
            detected_frames += 1

        detection_rate = (
            100.0 * detected_frames / total_frames if total_frames > 0 else 0.0
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
                f"detected={detection_rate:5.1f}% ({detected_frames}/{total_frames})",
                end="",
            )

        else:

            print(
                f"\rBall: not detected, "
                f"detected={detection_rate:5.1f}% ({detected_frames}/{total_frames})",
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
            f"({detected_frames}/{total_frames} frames, 'r' to reset)",
            (10, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            detection_color,
            1,
        )

        cv2.imshow(controls_window, control_display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q") or key == 27:
            break

        if key == ord("r"):
            total_frames = 0
            detected_frames = 0

    print()

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
