from pymavlink import mavutil
import time
import math

PORT = 14551
master = mavutil.mavlink_connection(f"udpin:0.0.0.0:{PORT}")
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected to sys", master.target_system)

# --- Helper Functions ---

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude 
    dNorth and dEast metres from the specified original_location.
    """
    earth_radius = 6378137.0 # Radius of "spherical" earth
    
    # Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return mavutil.mavlink.MAVLink_global_position_int_message(
        0, newlat*1e7, newlon*1e7, original_location.alt, 0, 0, 0, 0, 0
    )

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

def clear_mission():
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    t0 = time.time()
    while time.time() - t0 < 1.0:
        if master.recv_match(type="MISSION_ACK", blocking=False): break
        time.sleep(0.05)

def upload_pull(items):
    print(f"Uploading {len(items)} items")
    master.mav.mission_count_send(master.target_system, master.target_component, len(items))
    sent = 0
    while sent < len(items):
        req = master.recv_match(type=["MISSION_REQUEST_INT","MISSION_REQUEST"], blocking=True, timeout=10)
        if not req: raise RuntimeError("Timeout waiting for MISSION_REQUEST_INT")
        i = req.seq
        it = items[i]
        
        # Determine if we have int (1e7) or float coords
        lat = int(it["lat"]*1e7) if isinstance(it["lat"], float) else int(it["lat"])
        lon = int(it["lon"]*1e7) if isinstance(it["lon"], float) else int(it["lon"])

        master.mav.mission_item_int_send(
            master.target_system, master.target_component,
            i, it["frame"], it["cmd"], it["current"], it["autocontinue"],
            it["p1"], it["p2"], it["p3"], it["p4"],
            lat, lon, it["alt"]
        )
        sent += 1
    ack = master.recv_match(type="MISSION_ACK", blocking=True, timeout=10)
    print(" <- MISSION_ACK:", getattr(ack, "type", None))

def wait_pos(timeout=60):
    print("Waiting for GPS Lock...")
    t0 = time.time()
    while time.time() - t0 < timeout:
        m = master.recv_match(blocking=False)
        if not m: time.sleep(0.05); continue
        if m.get_type() == "GPS_RAW_INT" and m.fix_type >= 3:
            print(f"GPS Locked: {m.lat/1e7}, {m.lon/1e7}")
            return True
    return False

# --- Main Execution ---

# 1. Wait for valid position so we have a "Home" to calculate from
if not wait_pos(): raise RuntimeError("No GPS fix - cannot calculate relative coordinates")

# 2. Get the specific HOME position (or just use current GPS pos)
print("Requesting Home Position...")
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, 0, 0,0,0,0,0,0,0)
home_msg = master.recv_match(type='HOME_POSITION', blocking=True)
home_loc = mavutil.mavlink.MAVLink_global_position_int_message(
    0, home_msg.latitude, home_msg.longitude, home_msg.altitude, 0, 0, 0, 0, 0
)

# Convert integer coords (1e7) back to float degrees for the math function
home_loc.lat = home_msg.latitude / 1e7
home_loc.lon = home_msg.longitude / 1e7

print(f"Home set at: {home_loc.lat}, {home_loc.lon}")

# 3. Define Relative Waypoints (North m, East m, Alt m)
#    (0,0 is the Home Position)
relative_wps = [
    {"n": 50,  "e": 0,   "alt": 30},  # WP1: 50m North
    {"n": 50,  "e": 50,  "alt": 30},  # WP2: 50m North, 50m East
    {"n": 0,   "e": 50,  "alt": 30},  # WP3: 50m East (Back towards home)
    {"n": 0,   "e": 0,   "alt": 30},  # WP4: Return to Home (0,0)
]

# 4. Convert Relative -> Global
# 4. Convert Relative -> Global AND Add Land
mission_list = []

# Loop through relative points
for wp in relative_wps:
    # Do the math to get new Lat/Lon
    global_point = get_location_metres(home_loc, wp["n"], wp["e"])
    
    mission_list.append(dict(
        frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        cmd=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        current=0, autocontinue=1, p1=0, p2=0, p3=0, p4=0,
        lat=global_point.lat/1e7,
        lon=global_point.lon/1e7,
        alt=wp["alt"]
    ))

# --- NEW: Append the LAND command at the end ---
# We use the Home Location coordinates for the landing target
mission_list.append(dict(
    frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    cmd=mavutil.mavlink.MAV_CMD_NAV_LAND,
    current=0, autocontinue=1, p1=0, p2=0, p3=0, p4=0,
    lat=home_loc.lat, # Land exactly at Home Lat
    lon=home_loc.lon, # Land exactly at Home Lon
    alt=0             # Altitude 0 (Ground)
))
# 5. Clear and Upload
clear_mission()
upload_pull(mission_list)

# 6. Set Mode GUIDED, Arm, Takeoff
set_mode("GUIDED")
arm(True)
time.sleep(1.0)

print("Taking off to 30m...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
    0,0,0,0, 0,0, 30
)

# Wait for altitude (simple check)
while True:
    m = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
    if m.relative_alt >= 28000: # 28m
        print("Altitude reached!")
        break

# 7. Start Mission
print("Switching to AUTO to fly relative path...")
master.mav.mission_set_current_send(master.target_system, master.target_component, 0)
set_mode("AUTO")

while True:
    m = master.recv_match(type="MISSION_CURRENT", blocking=True)
    print(f"Current WP: {m.seq}")