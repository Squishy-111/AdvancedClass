from pymavlink import mavutil
import time

PORT = 14551  # Python's UDP port
master = mavutil.mavlink_connection(f"udpin:0.0.0.0:{PORT}")
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected to sys", master.target_system)

def set_mode(name):
    mode_id = master.mode_mapping()[name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

def arm(arm_it=True):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1 if arm_it else 0, 0, 0, 0, 0, 0, 0
    )

# --- three waypoints (no TAKEOFF)
mission = [
    dict(frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
         cmd=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
         current=0, autocontinue=1, p1=0, p2=0, p3=0, p4=0,
         lat=53.523219, lon=-113.526319, alt=30),
    dict(frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
         cmd=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
         current=0, autocontinue=1, p1=0, p2=0, p3=0, p4=0,
         lat=53.522900, lon=-113.525600, alt=30),
    dict(frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
         cmd=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
         current=0, autocontinue=1, p1=0, p2=0, p3=0, p4=0,
         lat=53.527735, lon=-113.522309, alt=30),
]

def clear_mission():
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    t0 = time.time()
    while time.time() - t0 < 1.0:
        if master.recv_match(type="MISSION_ACK", blocking=False): break
        time.sleep(0.05)

def upload_pull(items):
    print(f"Uploading {len(items)} items (WPs only)")
    master.mav.mission_count_send(master.target_system, master.target_component, len(items))
    sent = 0
    while sent < len(items):
        req = master.recv_match(type=["MISSION_REQUEST_INT","MISSION_REQUEST"], blocking=True, timeout=10)
        if not req: raise RuntimeError("Timeout waiting for MISSION_REQUEST_INT")
        i = req.seq
        it = items[i]
        master.mav.mission_item_int_send(
            master.target_system, master.target_component,
            i, it["frame"], it["cmd"], it["current"], it["autocontinue"],
            it["p1"], it["p2"], it["p3"], it["p4"],
            int(it["lat"]*1e7), int(it["lon"]*1e7), it["alt"]
        )
        sent += 1
    ack = master.recv_match(type="MISSION_ACK", blocking=True, timeout=10)
    print("  <- MISSION_ACK:", getattr(ack, "type", None))

def wait_pos(timeout=20):
    t0 = time.time()
    while time.time() - t0 < timeout:
        m = master.recv_match(blocking=False)
        if not m: time.sleep(0.05); continue
        t = m.get_type()
        if t == "GLOBAL_POSITION_INT": return True
        if t == "GPS_RAW_INT" and getattr(m,"fix_type",0) >= 3: return True
    return False

def readback_count():
    master.mav.mission_request_list_send(master.target_system, master.target_component)
    cnt = master.recv_match(type="MISSION_COUNT", blocking=True, timeout=3)
    print("Vehicle reports mission items:", getattr(cnt,"count", "?"))
    return getattr(cnt,"count",0)

# ---- sequence
clear_mission()
upload_pull(mission)
readback_count()

# GUIDED, wait for EKF/GPS, arm
set_mode("GUIDED")
if not wait_pos(): raise RuntimeError("No position estimate yet")
arm(True)
time.sleep(1.0)

# GUIDED takeoff to 30 m (param7=alt m AGL)
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
    0,0,0,0, 0,0, 30
)

# (Optional) wait until ~29 m rel alt before AUTO
t0 = time.time()
while time.time() - t0 < 25:
    m = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
    if m and getattr(m,"relative_alt",0) >= 29000:  # mm
        break
    time.sleep(0.05)

# Start mission at WP index 0 (first WP)
master.mav.mission_set_current_send(master.target_system, master.target_component, 0)
set_mode("AUTO")
print("AUTO engaged; press Ctrl+C to exit.")

while True:
    m = master.recv_match(blocking=False)
    if m and m.get_type() in ("HEARTBEAT","GLOBAL_POSITION_INT","MISSION_CURRENT"):
        print(m)
    time.sleep(0.05)
