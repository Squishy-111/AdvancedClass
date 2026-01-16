#!/usr/bin/env python3
# ultra_simple_servo_mapper.py

import time
import serial
from pymavlink import mavutil
PORT          = "/dev/cu.usbserial-DN04T9FH" 
BAUD          = 57600
TARGET_SYS    = 1
TARGET_COMP   = 1
SERVO_CHANNELS = [1, 2, 3, 4, 5, 6, 7, 8]     # Channels to test - each servo is mapped to a channel, see which channel moves which servo

PWM_MIN      = 1000
PWM_NEUTRAL  = 1500
PWM_MAX      = 2000
DWELL        = 0.5   # Seconds between move_servo calls 


def open_serial(port, baud):
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=1)
        print(f"[OK] Opened {port} @ {baud}")
        return ser
    except Exception as e:
        print(f"[ERR] Serial error: {e}")
        return None

def send_servo(ser, servo_num, pwm):
    # Send MAV_CMD_DO_SET_SERVO
    mav = mavutil.mavlink.MAVLink(ser)
    mav.srcSystem = 255
    mav.srcComponent = 190
    msg = mav.command_long_encode(
        TARGET_SYS, TARGET_COMP,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        float(servo_num), float(pwm),
        0, 0, 0, 0, 0
    )
    ser.write(msg.pack(mav))
    ser.flush()

def pulse_channel(ser, ch):
    print(f"\n--- Channel {ch} ---")
    try:
        print(f"  MIN  -> {PWM_MIN}")
        send_servo(ser, ch, PWM_MIN);     time.sleep(DWELL)
        print(f"  NEUT -> {PWM_NEUTRAL}")
        send_servo(ser, ch, PWM_NEUTRAL); time.sleep(DWELL)
        print(f"  MAX  -> {PWM_MAX}")
        send_servo(ser, ch, PWM_MAX);     time.sleep(DWELL)
        print(f"  NEUT -> {PWM_NEUTRAL}")
        send_servo(ser, ch, PWM_NEUTRAL); time.sleep(DWELL)
    except Exception as e:
        print(f"[ERR] Channel {ch} pulse failed: {e}")

def main():
    ser = open_serial(PORT, BAUD)
    if not ser:
        return
    try:
        for ch in SERVO_CHANNELS:
            # start neutral
            pulse_channel(ser, ch)
    finally:
        ser.close()
        print("[DONE] Serial closed. All tested channels set to NEUTRAL.")

if __name__ == "__main__":
    main()
