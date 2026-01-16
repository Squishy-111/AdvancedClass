import time
import numpy as np
import cv2
from pymavlink import mavutil

# ==============================
# Configuration
# ==============================
# 0.2 radians is approx 11.5 degrees. 
# This tells the drone the target is ~11 degrees to the Right and Forward.
FIXED_ANGLE_X = 0.2  
FIXED_ANGLE_Y = 0.2
FIXED_DISTANCE = 5.0 # Tell the drone the target is 5m away

# ==============================
# Connect to vehicle
# ==============================
def connect_mavlink():
    # Listening to the Docker container on port 14552
    connection = mavutil.mavlink_connection('udpin:0.0.0.0:14552')
    connection.wait_heartbeat()
    print("Heartbeat received from system %u component %u" %
          (connection.target_system, connection.target_component))
    return connection

# ==============================
# MAVLink send function
# ==============================
def SendLandingTarget(connection, distance, angle_x, angle_y):
    """Send a MAVLink LANDING_TARGET message."""
    msg = connection.mav.landing_target_encode(
        int(time.time() * 1000) & 0xFFFFFFFF,  # time_boot_ms
        0,                                     # target_num
        mavutil.mavlink.MAV_FRAME_BODY_FRD,    # frame
        angle_x,
        angle_y,
        distance,
        0.0, # size_x (ignored)
        0.0, # size_y (ignored)
    )
    connection.mav.send(msg)

# ==============================
# Main loop
# ==============================
def main():
    conn = connect_mavlink()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        return

    try:
        while True:
            ret, img = cap.read()
            if not ret:
                print("Cannot receive image (stream end?)")
                break

            # ============================================
            # BYPASSING VISION LOGIC
            # ============================================
            # We ignore the image and simply send the fixed offsets defined at the top.
            
            print(f"Sending FIXED OFFSET: AngX={np.degrees(FIXED_ANGLE_X):.1f}° | AngY={np.degrees(FIXED_ANGLE_Y):.1f}°")
            SendLandingTarget(conn, FIXED_DISTANCE, FIXED_ANGLE_X, FIXED_ANGLE_Y)

            # Draw a warning on screen so you know it's a simulation
            cv2.putText(img, "SIMULATING CONSTANT OFFSET", (50, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(img, "Drone should drift Forward-Right", (50, 90), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Display window (just to keep the script running smoothly)
            cv2.imshow('Original', img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # 10Hz update rate
            time.sleep(0.1)

    finally:
        cap.release()
        cv2.destroyAllWindows()

# ==============================
# Run
# ==============================
if __name__ == "__main__":
    main()