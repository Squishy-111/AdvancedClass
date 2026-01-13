# servo_hat_tracker.py
from ultralytics import YOLO
import cv2
import serial
from pymavlink import mavutil

# ==== TUNABLES ====
CONF_MIN = 0.60          # only accept detections >= 60% confidence
TARGET_LABEL = "hat"     # set to None to accept any class
# ===================

# MAVLink helpers
def open_serial(port="/dev/tty.Bluetooth-Incoming-Port", baud=57600):
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=1)
        print(f"Opened serial port {port} @ {baud}")
        return ser
    except Exception as e:
        print(f"Serial error: {e}")
        return None

def move_servo(ser, servo_num, pwm, target_sys=1, target_comp=1):
    mav = mavutil.mavlink.MAVLink(ser)
    mav.srcSystem = 1
    mav.srcComponent = 1
    msg = mav.command_long_encode(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        float(servo_num), float(pwm),
        0, 0, 0, 0, 0
    )
    ser.write(msg.pack(mav)); ser.flush()
    # print(f"Servo {servo_num} -> {pwm}")  # Noisy if left on; uncomment for debugging

def main():
    # Serial link to FC (close QGC/Mission Planner if using same port)
    ser = open_serial("/dev/tty.Bluetooth-Incoming-Port", 57600)
    if not ser:
        return

    # Load YOLO model
    model = YOLO("/opt/homebrew/runs/detect/train/weights/best.pt")  # Update path if needed

    # Resolve class id for TARGET_LABEL (if provided)
    TARGET_CLS = None
    if TARGET_LABEL is not None:
        names = getattr(getattr(model, "model", model), "names", {})
        TARGET_CLS = next((i for i, n in names.items() if str(n).lower() == TARGET_LABEL.lower()), None)
        if TARGET_CLS is None:
            print(f"[warn] TARGET_LABEL '{TARGET_LABEL}' not found in model.names; accepting any class.")

    # Open webcam
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Failed to open camera.")
        ser.close()
        return

    SERVO_RUDDER   = 4   # yaw/left-right (horizontal error)
    SERVO_ELEVATOR = 5   # pitch/up-down (vertical error)

    GAIN_X = 1.2         # pixels -> PWM slope for rudder
    GAIN_Y = 1.2         # pixels -> PWM slope for elevator
    DB_X   = 20          # deadband (pixels) for rudder
    DB_Y   = 20          # deadband (pixels) for elevator

    PWM_MIN, PWM_MAX = 1000, 2000
    NEUTRAL          = 1500

    prev_pwm_rudder  = NEUTRAL
    prev_pwm_elev    = NEUTRAL

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            # Run YOLO (quiet)
            results = model(frame, stream=True, verbose=False)

            # Pick the most confident detection that passes filters
            best, best_conf, best_cls = None, -1.0, None
            for r in results:
                if r.boxes is None:
                    continue
                for box in r.boxes:
                    conf = float(box.conf[0])
                    if conf < CONF_MIN:
                        continue
                    cls = int(box.cls[0]) if box.cls is not None else -1
                    if TARGET_CLS is not None and cls != TARGET_CLS:
                        continue
                    if conf > best_conf:
                        best_conf, best, best_cls = conf, box, cls

            if best is not None:
                x1, y1, x2, y2 = map(int, best.xyxy[0])
                cx, cy = (x1 + x2)//2, (y1 + y2)//2

                # Draw detection (with label+conf)
                cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
                label_txt = f"{TARGET_LABEL or 'obj'} {best_conf:.2f}"
                cv2.putText(frame, label_txt, (x1, max(20, y1-8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                cv2.circle(frame, (cx,cy), 4, (0,255,0), -1)

                # Image center
                h, w = frame.shape[:2]
                center_x = w // 2
                center_y = h // 2

                # Errors (pixels): right/down positive
                err_x = cx - center_x   # horizontal error -> rudder (servo 4)
                err_y = cy - center_y   # vertical error   -> elevator (servo 5)

                # Map to PWM with deadband & clamp
                if abs(err_x) < DB_X:
                    pwm_rudder = NEUTRAL
                else:
                    pwm_rudder = int(NEUTRAL - err_x * GAIN_X)  # flip sign if needed
                pwm_rudder = max(PWM_MIN, min(PWM_MAX, pwm_rudder))

                if abs(err_y) < DB_Y:
                    pwm_elev = NEUTRAL
                else:
                    pwm_elev = int(NEUTRAL - err_y * GAIN_Y)    # flip sign if needed
                pwm_elev = max(PWM_MIN, min(PWM_MAX, pwm_elev))

                # Send only on change
                if pwm_rudder != prev_pwm_rudder:
                    move_servo(ser, SERVO_RUDDER, pwm_rudder)
                    prev_pwm_rudder = pwm_rudder

                if pwm_elev != prev_pwm_elev:
                    move_servo(ser, SERVO_ELEVATOR, pwm_elev)
                    prev_pwm_elev = pwm_elev

            else:
                # Auto center when no acceptable detection
                if prev_pwm_rudder != NEUTRAL:
                    move_servo(ser, SERVO_RUDDER, NEUTRAL); prev_pwm_rudder = NEUTRAL
                if prev_pwm_elev != NEUTRAL:
                    move_servo(ser, SERVO_ELEVATOR, NEUTRAL); prev_pwm_elev = NEUTRAL

            # HUD
            h, w = frame.shape[:2]
            cv2.line(frame, (w//2, 0), (w//2, h), (255,255,255), 1)  # vertical center line
            cv2.line(frame, (0, h//2), (w, h//2), (255,255,255), 1)  # horizontal center line
            cv2.putText(frame, f"Rudder S4 PWM {prev_pwm_rudder}", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(frame, f"Elev   S5 PWM {prev_pwm_elev}", (10,60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

            cv2.imshow("Hat Tracker: Rudder (S4) + Elevator (S5)", frame)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()
        try:
            # Center on exit
            move_servo(ser, SERVO_RUDDER, NEUTRAL)
            move_servo(ser, SERVO_ELEVATOR, NEUTRAL)
        except Exception:
            pass
        ser.close()
        print("Exited — camera and serial closed.")

if __name__ == "__main__":
    main()
