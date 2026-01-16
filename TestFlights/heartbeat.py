#!/usr/bin/env python3
from pymavlink import mavutil

PORT = "/dev/cu.usbserial-DN04T9FH"   # <-- change this to your cu.* port

for baud in [115200, 57600, 921600]:
    print(f"Trying {PORT} @ {baud} ...")
    m = mavutil.mavlink_connection(PORT, baud=baud)
    hb = m.wait_heartbeat(timeout=5)
    if hb:
        print(f"✅ Heartbeat OK at {baud}. sys={m.target_system} comp={m.target_component}")
        break
    else:
        print("❌ No heartbeat")
else:
    raise SystemExit("No heartbeat at any baud. Likely not MAVLink on this port.")
