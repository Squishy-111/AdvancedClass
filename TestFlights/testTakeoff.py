#!/usr/bin/env python3
import time
from pymavlink import mavutil

def cmd(m, command, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        command, 0,
        float(p1), float(p2), float(p3), float(p4), float(p5), float(p6), float(p7)
    )

def try_arm(m, timeout=10):
    print("Arming...")
    cmd(m, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1)

    t0 = time.time()
    while time.time() - t0 < timeout:
        # STATUSTEXT will include messages like "Roll (RC1) is not neutral"
        msg = m.recv_match(type=["STATUSTEXT", "HEARTBEAT"], blocking=True, timeout=1)
        if not msg:
            continue

        if msg.get_type() == "STATUSTEXT":
            print("STATUS:", msg.text)

        if msg.get_type() == "HEARTBEAT":
            # Check armed flag in heartbeat base_mode
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("ARMED")
                return True

    print("Failed to arm (timed out).")
    return False

def main():
    port = "udpin:0.0.0.0:14551"   # <-- change from udp:127.0.0.1:14551
    target_alt = float(input("Takeoff altitude (m): ").strip() or "3.0")
    hover_sec  = float(input("Hover time (sec): ").strip() or "5")

    print(f"Connecting to {port} ...")
    m = mavutil.mavlink_connection(port)
    m.wait_heartbeat(timeout=15)
    print(f"Heartbeat: sys={m.target_system} comp={m.target_component}")

    print("Mode: GUIDED")
    m.set_mode("GUIDED")
    time.sleep(1)

    if not try_arm(m, timeout=12):
        return  # stop if it won't arm

    print(f"Takeoff to {target_alt} m...")
    cmd(m, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=target_alt)

    time.sleep(hover_sec)

    print("Landing...")
    cmd(m, mavutil.mavlink.MAV_CMD_NAV_LAND)

    time.sleep(20)

    print("Disarming...")
    cmd(m, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=0)
    print("Done.")

if __name__ == "__main__":
    main()
