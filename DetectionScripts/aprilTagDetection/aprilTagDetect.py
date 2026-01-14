import cv2
from pupil_apriltags import Detector

def main():
    cap = cv2.VideoCapture(0)  # Maybe AI gen video file for overhead simulation (video file path)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera")

    # Create detector
    at_detector = Detector(
        families="tag36h11",   # This needs to match whatever family of tag you printed. 36h11 is most common
        nthreads=2,            # For a RPi, shouldnt exceed 2 threads.
        quad_decimate=2.0,     # Downsampling. Higher number means faster processing but worse image quality...
        quad_sigma=0.0,        # Blur, usually 0
        refine_edges=1,        # Something about increased edge refinement at full quality. Shouldnt change this
        decode_sharpening=0.25 # Higher can amplify noise. I wouldnt mess with this too mucn either
    )

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Apriltag detection is based on intensity, not colour. Greyscale is more suitable

        # Detect tags
        detections = at_detector.detect(gray)

        for det in detections:
            # det.tag_id, det.center, det.corners
            cx, cy = int(det.center[0]), int(det.center[1]) # Center point
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

            corners = det.corners.astype(int)
            # Draw 4 lines, each line connects two adjacent corners to form a square
            for i in range(4):
                p1 = tuple(corners[i])
                p2 = tuple(corners[(i+1) % 4])
                cv2.line(frame, p1, p2, (255, 0, 0), 10)

            cv2.putText(frame, f"id={det.tag_id}", (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("apriltags", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
