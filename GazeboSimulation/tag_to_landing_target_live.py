import time, math
import cv2
from pupil_apriltags import Detector
from pymavlink import mavutil

URL = "http://127.0.0.1:8090/feed.mjpg"

# Rough intrinsics (good enough to start)
HFOV_DEG = 60.0
RATE_HZ = 20.0

# Connect to SITL
m = mavutil.mavlink_connection("udp:127.0.0.1:14550")
m.wait_heartbeat()
print("MAVLink heartbeat OK")

det = Detector(families="tag36h11", nthreads=2, quad_decimate=1.0, refine_edges=True)

cap = cv2.VideoCapture(URL)
if not cap.isOpened():
    raise SystemExit("Could not open MJPEG stream (is ffmpeg running?)")

# Frame size -> intrinsics approximation
ok, frame = cap.read()
if not ok:
    raise SystemExit("Could not read initial frame")
h, w = frame.shape[:2]
cx, cy = w / 2.0, h / 2.0
fx = (w / 2.0) / math.tan(math.radians(HFOV_DEG) / 2.0)
fy = fx

print(f"Frame {w}x{h}; fx~{fx:.1f}; cx={cx:.1f} cy={cy:.1f}")

dt = 1.0 / RATE_HZ
last_print = 0.0

while True:
    ok, frame = cap.read()
    if not ok:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = det.detect(gray)

    cv2.putText(frame, f"tags: {len(tags)}", (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,0,255), 2)

    if tags:
        # pick best by decision margin
        t = max(tags, key=lambda x: x.decision_margin)

        u, v = t.center
        pts = t.corners.astype(int)
        cv2.polylines(frame, [pts], True, (0,255,0), 2)
        cv2.circle(frame, (int(u), int(v)), 5, (255,0,0), -1)
        cv2.putText(frame, f"id={t.tag_id} dm={t.decision_margin:.1f}",
                    (int(u)+10, int(v)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        # Pixel -> angles (pinhole)
        ax = math.atan((u - cx) / fx)
        ay = math.atan((v - cy) / fy)

        # One of these may need flipping depending on camera frame.
        # If vehicle corrects the wrong way, flip ONE at a time:
        # ax = -ax
        # ay = -ay

        # Send LANDING_TARGET (angles-only)
        m.mav.landing_target_send(
            int(time.time() * 1e6),
            0,     # target_num
            12,    # MAV_FRAME_BODY_FRD
            float(ax),
            float(ay),
            0.0,   # distance unknown (0 is fine)
            0.0, 0.0,
            0.0, 0.0, 0.0,
            (0.0, 0.0, 0.0, 1.0),
            2,     # type: fiducial marker
            0      # position_valid
        )

        now = time.time()
        if now - last_print > 1.0:
            print(f"LANDING_TARGET ax={ax:+.3f} ay={ay:+.3f} px=({u:.1f},{v:.1f}) dm={t.decision_margin:.1f}")
            last_print = now

    cv2.imshow("Tag -> LANDING_TARGET", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

    time.sleep(dt)

cap.release()
cv2.destroyAllWindows()
