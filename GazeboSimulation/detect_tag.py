import cv2
from pupil_apriltags import Detector

URL = "http://127.0.0.1:8090/feed.mjpg"

det = Detector(
    families="tag36h11",
    nthreads=2,
    quad_decimate=1.0,
    refine_edges=True,
    decode_sharpening=0.25,
)

cap = cv2.VideoCapture(URL)
print("Opened:", cap.isOpened())
if not cap.isOpened():
    raise SystemExit("Could not open MJPEG stream. Is ffmpeg running on :8090?")

while True:
    ok, frame = cap.read()
    if not ok:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = det.detect(gray)

    # Always show whether we are detecting
    cv2.putText(frame, f"tags: {len(tags)}",
                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,0,255), 2)

    for t in tags:
        u, v = t.center
        pts = t.corners.astype(int)
        cv2.polylines(frame, [pts], True, (0,255,0), 2)
        cv2.circle(frame, (int(u), int(v)), 5, (255,0,0), -1)
        cv2.putText(frame, f"id={t.tag_id}",
                    (int(u)+10, int(v)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

        # Print occasionally so you can tell it works even if overlay is subtle
        print(f"DETECTED id={t.tag_id} center=({u:.1f},{v:.1f}) decision={t.decision_margin:.2f}")


    cv2.imshow("Detect + View", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
