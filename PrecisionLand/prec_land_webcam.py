#!/usr/bin/env python3
import time
import numpy as np
import cv2
from pymavlink import mavutil

# ==============================
# Camera Configuration
# ==============================
HORIZONTAL_RESOLUTION = 640  # in pixels
VERTICAL_RESOLUTION = 480    # in pixels
HORIZONTAL_FOV = np.radians(60)  # not used directly but kept for reference
VERTICAL_FOV = np.radians(45)





def GetCoordsFromImage(img):
    """
    Detect a purple, square-ish blob and compute:
      distance, angle_x, angle_y, size_x, size_y

    Returns:
      (distance, angle_x, angle_y, size_x, size_y), mask, target_vis
      OR (None, mask, black_image) if nothing found
    """
    # --- Preprocess ---
    blurred = cv2.GaussianBlur(img, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # --- Color threshold for purple ---
    # NOTE: you may still need to tweak these after looking at the "Mask" window.
    # H is [0..179] -> ~130–150 is purple/magenta-ish in many webcams.
    lower_purple = np.array([130, 60, 40])   # H, S, V
    upper_purple = np.array([155, 255, 255])

    mask = cv2.inRange(hsv, lower_purple, upper_purple)

    # Clean up the mask
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # --- Find contours ---
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask, np.zeros_like(img)

    # Pick the "best" square-like contour
    best_contour = None
    best_area = 0

    for c in contours:
        area = cv2.contourArea(c)
        if area < 500:  # ignore tiny blobs
            continue

        x, y, w, h = cv2.boundingRect(c)
        aspect_ratio = w / float(h)
        if not (0.8 <= aspect_ratio <= 1.2):
            # not square-ish enough
            continue

        # how much of the bounding box is filled?
        rect_area = w * h
        extent = area / float(rect_area)
        if extent < 0.6:
            # too skinny / lots of holes
            continue

        # polygon approximation: look for ~4 corners
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        if len(approx) < 4 or len(approx) > 6:
            # not very rectangle-like
            continue

        # If we get here, this contour is "square-ish"
        if area > best_area:
            best_area = area
            best_contour = c

    if best_contour is None:
        return None, mask, np.zeros_like(img)

    # --- Compute center and size of the chosen contour ---
    x, y, w, h = cv2.boundingRect(best_contour)
    x_mean = x + w // 2
    y_mean = y + h // 2
    x_width = w
    y_width = h

    # Visualization
    target_vis = img.copy()
    cv2.rectangle(target_vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
    s = 10
    cv2.line(target_vis, (x_mean - s, y_mean - s), (x_mean + s, y_mean + s), (0, 255, 0), 2)
    cv2.line(target_vis, (x_mean - s, y_mean + s), (x_mean + s, y_mean - s), (0, 255, 0), 2)

    # --- Distance heuristic (same idea as before, but a bit simpler) ---
    img_width = img.shape[1]
    norm_size = x_width / float(img_width)  # 0..1

    if norm_size < 0.05:
        distance = 4.0
    elif norm_size > 0.5:
        distance = 0.5
    else:
        distance = 4.0 - 7.0 * (norm_size - 0.05) / (0.5 - 0.05)
        distance = np.clip(distance, 0.5, 4.0)

    # --- Angle computation (same as before) ---
    cx = img.shape[1] / 2.0
    cy = img.shape[0] / 2.0

    dx = (x_mean - cx)      # +right
    dy = (cy - y_mean)      # +forward (invert y)

    fx = img.shape[1] / (2.0 * np.tan(HORIZONTAL_FOV / 2.0))
    fy = img.shape[0] / (2.0 * np.tan(VERTICAL_FOV / 2.0))

    angle_x = np.arctan2(dx, fx)
    angle_y = np.arctan2(dy, fy)

    size_x = 2.0 * np.arctan2(x_width / 2.0, fx)
    size_y = 2.0 * np.arctan2(y_width / 2.0, fy)

    return (distance, angle_x, angle_y, size_x, size_y), mask, target_vis





# ==============================
# Connect to vehicle
# ==============================
def connect_mavlink():
    # NOTE: match this to your SITL --out port!
    # If you followed the localhost setup:
    #   sim_vehicle.py ... --out=udp:127.0.0.1:14541
    conn = mavutil.mavlink_connection('udpin:0.0.0.0:14541')
    conn.wait_heartbeat()
    print(
        "Heartbeat received from system %u component %u"
        % (conn.target_system, conn.target_component)
    )
    return conn

# ==============================
# Image processing and coordinate estimation
# ==============================
def GetCoordsFromImage(img):
    """
    Detect a purple-ish square-ish blob and compute:
      distance, angle_x, angle_y, size_x, size_y

    Returns:
      (distance, angle_x, angle_y, size_x, size_y), mask, target_vis
      OR (None, mask, black_image) if nothing found
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # ---- Color threshold for purple ----
    # OpenCV Hue is [0..179] representing [0..360) degrees * 0.5.
    # Purple/violet-ish region roughly around 260–300 deg -> 130–150 here.
    # Tweak these while watching the Mask window if needed.
    lower_hsv = np.array([130, 50, 50])  # H,S,V
    upper_hsv = np.array([155, 255, 255])
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

    # Optional morphological cleanup
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    num_labels, labels = cv2.connectedComponents(mask)
    if num_labels <= 1:
        # only background
        return None, mask, np.zeros_like(img)

    counts = np.bincount(labels.flatten())
    counts[0] = 0  # ignore background

    # Basic "blob exists?" check
    if np.max(counts) < 50:
        return None, mask, np.zeros_like(img)

    # Take the largest blob
    largest_label = np.argmax(counts)
    large_blob_mask = (labels == largest_label).astype(np.uint8) * 255

    # Optional: refine with contour & "square-ish" filter
    contours, _ = cv2.findContours(
        large_blob_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return None, mask, np.zeros_like(img)

    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < 100:  # very small blob, ignore
        return None, mask, np.zeros_like(img)

    x, y, w, h = cv2.boundingRect(c)
    aspect_ratio = w / float(h)

    # Rough "square-ish": aspect ratio between 0.7 and 1.3
    if not (0.7 <= aspect_ratio <= 1.3):
        # Not very square, skip
        return None, mask, np.zeros_like(img)

    # Compute blob center & size in image coords
    x_mean = x + w // 2
    y_mean = y + h // 2
    x_width = w
    y_width = h

    # Visualization image
    target_vis = img.copy()
    cv2.rectangle(target_vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
    s = 10
    cv2.line(target_vis, (x_mean - s, y_mean - s), (x_mean + s, y_mean + s), (0, 255, 0), 2)
    cv2.line(target_vis, (x_mean - s, y_mean + s), (x_mean + s, y_mean - s), (0, 255, 0), 2)

    # ---- Estimate distance (simple heuristic) ----
    # Bigger blob -> closer. This is just a monotonic model, not physically accurate.
    img_width = img.shape[1]
    norm_size = x_width / float(img_width)  # 0..1

    # Map size to distance: tuned so you get ~0.5–4 m
    # (feel free to tweak these numbers while watching the console)
    if norm_size < 0.05:
        distance = 4.0
    elif norm_size > 0.5:
        distance = 0.5
    else:
        # linear-ish mapping
        distance = 4.0 - 7.0 * (norm_size - 0.05) / (0.5 - 0.05)
        distance = np.clip(distance, 0.5, 4.0)

    # ---- Compute angles (radians) ----
    # Coordinate system: LANDING_TARGET expects angles in BODY_FRD frame,
    # where x is right, y is forward.
    # Here we treat:
    #  - +angle_x when target is to the RIGHT in the image
    #  - +angle_y when target is ABOVE (forward) in the image
    cx = img.shape[1] / 2.0
    cy = img.shape[0] / 2.0

    dx = (x_mean - cx)  # +right
    dy = (cy - y_mean)  # +forward (flip vertical axis)

    # Small angle approximation: angle ≈ offset / focal_length
    # We approximate focal_length in pixels via FOV.
    fx = img.shape[1] / (2.0 * np.tan(HORIZONTAL_FOV / 2.0))
    fy = img.shape[0] / (2.0 * np.tan(VERTICAL_FOV / 2.0))

    angle_x = np.arctan2(dx, fx)  # radians
    angle_y = np.arctan2(dy, fy)  # radians

    # Apparent angular size
    size_x = 2.0 * np.arctan2(x_width / 2.0, fx)
    size_y = 2.0 * np.arctan2(y_width / 2.0, fy)

    return (distance, angle_x, angle_y, size_x, size_y), mask, target_vis

# ==============================
# MAVLink send function
# ==============================
def SendLandingTarget(connection, distance, angle_x, angle_y, size_x, size_y):
    """Send a MAVLink LANDING_TARGET message."""
    msg = connection.mav.landing_target_encode(
        int(time.time() * 1000) & 0xFFFFFFFF,  # time_boot_ms
        0,                                     # target_num
        mavutil.mavlink.MAV_FRAME_BODY_FRD,    # frame
        float(angle_x),
        float(angle_y),
        float(distance),
        float(size_x),
        float(size_y),
    )
    connection.mav.send(msg)

# ==============================
# Main loop
# ==============================
def main():
    conn = connect_mavlink()

    # 0 = default webcam; change to 1,2,... if you have multiple cameras
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        return

    # Optional: force resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, HORIZONTAL_RESOLUTION)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VERTICAL_RESOLUTION)

    try:
        while True:
            ret, img = cap.read()
            if not ret:
                print("Cannot receive image (stream end?)")
                break

            coords, mask, target_vis = GetCoordsFromImage(img)

            if coords:
                distance, angle_x, angle_y, size_x, size_y = coords
                print(
                    f"Target found: dist={distance:.2f} m, "
                    f"angle_x={np.degrees(angle_x):.2f}°, "
                    f"angle_y={np.degrees(angle_y):.2f}°"
                )
                SendLandingTarget(conn, distance, angle_x, angle_y, size_x, size_y)
            else:
                # No target detected this frame
                pass

            # Display windows for debugging
            cv2.imshow("Original", img)
            cv2.imshow("Mask", mask)
            cv2.imshow("Target", target_vis)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            time.sleep(0.1)  # ~10 Hz

    finally:
        cap.release()
        cv2.destroyAllWindows()

# ==============================
# Run
# ==============================
if __name__ == "__main__":
    main()
