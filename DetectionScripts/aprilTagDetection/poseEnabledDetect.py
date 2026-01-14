import cv2
import numpy as np
from pupil_apriltags import Detector

TAG_SIZE_M = 0.16  # 16 cm tag (black border square size)

# These are the camera intrinsics. Determined via Cam/Hassans script
FX, FY = 600.0, 600.0
CX, CY = 320.0, 240.0

def main():
    cap = cv2.VideoCapture(0)
    at_detector = Detector(families="tag36h11", nthreads=2, quad_decimate=2.0, refine_edges=1)

    camera_params = (FX, FY, CX, CY)

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = at_detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=TAG_SIZE_M
        )

        for det in detections:
            # Pose results:
            # det.pose_t is translation (x,y,z) in meters in camera frame
            # det.pose_R is a 3x3 rotation matrix, not used but we could if we want roll/pitch/yaw translation
            t = det.pose_t  # numpy array shape (3,1) or (3,)
            z_m = float(t[2])
            x_m = float(t[0])
            y_m = float(t[1])

            cx, cy = int(det.center[0]), int(det.center[1])
            cv2.putText(frame, f"id={det.tag_id} z={z_m:.2f}m",
                        (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

            # Draw outline
            corners = det.corners.astype(int)
            for i in range(4):
                cv2.line(frame, tuple(corners[i]), tuple(corners[(i+1)%4]), (255,0,0), 2)

        cv2.imshow("apriltag_pose", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
