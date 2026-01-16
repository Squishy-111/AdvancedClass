import cv2

pipeline = (
    "udpsrc port=5600 caps=application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 "
    "! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=true sync=false"
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
print("Opened:", cap.isOpened())

while True:
    ok, frame = cap.read()
    if not ok:
        continue
    cv2.imshow("Gazebo UDP stream", frame)
    if cv2.waitKey(1) & 0xFF == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()
