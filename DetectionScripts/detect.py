import cv2
import numpy as np
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
