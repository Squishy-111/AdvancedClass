#!/usr/bin/env python3
"""
Complete Precision Landing SITL Controller.
Combines: Parameter Setup + Mission Upload + Synthetic Vision Target Streaming.
"""

import math
import time
from pymavlink import mavutil

# --- CONFIGURATION ---
MAVLINK_URL = "udpin:0.0.0.0:14551"
HOME_LAT = 53.5269
HOME_LON = -113.5256
TARGET_LAT = 53.5271
TARGET_LON = -113.5254
TAKEOFF_ALT = 20.0 

def connect_mavlink():
    print(f"Connecting to {MAVLINK_URL} ...")
    m = mavutil.mavlink_connection(MAVLINK_URL)
    m.wait_heartbeat()
    print(f"Heartbeat from system {m.target_system} component {m.target_component}")
    return m

def set_param(m, name, value, ptype=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
    """Sets a parameter and waits for echo."""
    m.mav.param_set_send(
        m.target_system, m.target_component,
        name.encode('utf-8'), float(value), ptype
    )
    # Give it a moment to process
    time.sleep(0.1)

def setup_vehicle(m):
    """Configures PLND and RNGFND parameters based on your docs."""
    print("--- Setting Parameters ---")
    
    # Precision Landing Params
    # set_param(m, "PLND_ENABLED", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
    # set_param(m, "PLND_TYPE", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8)     # Companion Computer
    # set_param(m, "PLND_EST_TYPE", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT8) # Raw (no Kalman)
    
    # # Virtual Rangefinder (Required for SITL Landing)
    # # SITL needs these to simulate a bouncing sonar signal
    # set_param(m, "RNGFND1_TYPE", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8) # Analog
    # set_param(m, "RNGFND1_MIN_CM", 0)
    # set_param(m, "RNGFND1_MAX_CM", 4000)
    # set_param(m, "RNGFND1_PIN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
    # set_param(m, "RNGFND1_SCALING", 12.12)
    
    print("Parameters sent. (Note: In real hardware, reboot might be required)")

def upload_mission(m):
    """Uploads the mission items defined in your MAVWritetestmission.py."""
    print("--- Uploading Mission ---")
    
    mission_items = [
        # 0: Home/Dummy
        mavutil.mavlink.MAVLink_mission_item_int_message(
            m.target_system, m.target_component, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            1, 1, 0, 0, 0, 0, int(HOME_LAT*1e7), int(HOME_LON*1e7), 0
        ),
        # 1: Takeoff
        mavutil.mavlink.MAVLink_mission_item_int_message(
            m.target_system, m.target_component, 1,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 1, 0, 0, 0, 0, int(HOME_LAT*1e7), int(HOME_LON*1e7), int(TAKEOFF_ALT)
        ),
        # 2: Waypoint 1
        mavutil.mavlink.MAVLink_mission_item_int_message(
            m.target_system, m.target_component, 2,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1, 0, 0, 0, 0, int(53.5269 * 1e7), int(-113.5256 * 1e7), int(TAKEOFF_ALT)
        ),
        # 3: Waypoint 2 (Approach)
        mavutil.mavlink.MAVLink_mission_item_int_message(
            m.target_system, m.target_component, 3,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1, 0, 0, 0, 0, int(53.5271 * 1e7), int(-113.5260 * 1e7), int(TAKEOFF_ALT)
        ),
        # 4: Land (Target Location)
        mavutil.mavlink.MAVLink_mission_item_int_message(
            m.target_system, m.target_component, 4,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 1, 0, 0, 0, 0, int(TARGET_LAT * 1e7), int(TARGET_LON * 1e7), 0
        )
    ]

    m.mav.mission_clear_all_send(m.target_system, m.target_component)
    m.mav.mission_count_send(m.target_system, m.target_component, len(mission_items))

    for i, item in enumerate(mission_items):
        msg = m.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True)
        print(f"Uploading item {msg.seq}")
        m.mav.send(mission_items[msg.seq])

    ack = m.recv_match(type='MISSION_ACK', blocking=True)
    print(f"Mission ACK: {ack.type}")

def ll_to_local_m(lat, lon, lat0, lon0):
    """Calculates offset in meters from reference lat/lon."""
    R = 6378137.0
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    x_east = R * dlon * math.cos(math.radians((lat + lat0) / 2.0))
    y_north = R * dlat
    return x_east, y_north

def send_synthetic_target(m, drone_lat, drone_lon, drone_alt, drone_yaw):
    """
    Simulates the computer vision logic.
    Calculates where the target IS relative to drone, and sends that as if 
    it were a camera vector.
    """
    # 1. Get Vector from Drone to Target (North/East meters)
    ex, ny = ll_to_local_m(TARGET_LAT, TARGET_LON, drone_lat, drone_lon)

    # 2. Rotate into Body Frame (Forward/Right) based on Yaw
    # Forward (x) = Cos(yaw)*North + Sin(yaw)*East
    # Right (y)   = -Sin(yaw)*North + Cos(yaw)*East
    bx = math.cos(drone_yaw) * ny + math.sin(drone_yaw) * ex
    by = -math.sin(drone_yaw) * ny + math.cos(drone_yaw) * ex

    # 3. Calculate Angles (Camera Frame)
    # Note: MAVLink LANDING_TARGET uses "angle_x" (Right/Left) and "angle_y" (Up/Down)
    # We use atan2 to get the precise angle to the target
    if drone_alt < 0.2: 
        return # Too low to calculate

    angle_x = math.atan2(by, drone_alt) # Angle to the right
    angle_y = math.atan2(bx, drone_alt) # Angle forward

    dist = math.sqrt(bx*bx + by*by + drone_alt*drone_alt)

    # 4. FIX: Use the correct timestamp masking from your working code
    time_boot_ms = int(time.time() * 1000) & 0xFFFFFFFF

    m.mav.landing_target_send(
        time_boot_ms,                       # Fixed Timestamp
        0,                                  # target_num
        mavutil.mavlink.MAV_FRAME_BODY_FRD, # frame
        angle_x,                            # angle_x (rad)
        angle_y,                            # angle_y (rad)
        dist,                               # distance (m)
        0.0, 0.0                            # size_x, size_y (ignored by ArduPilot usually)
    )
    
    return dist, math.degrees(angle_x), math.degrees(angle_y)

def main():
    m = connect_mavlink()
    setup_vehicle(m)
    upload_mission(m)

    print("--- Arming and Starting Mission ---")
    m.set_mode('STABILIZE')
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    m.motors_armed_wait()
    print("ARMED")

    m.set_mode('AUTO')
    m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)
    print("AUTO MODE ENGAGED")

    # Main Loop: Listen for position, send target if landing
    while True:
        msg = m.recv_match(type=['GLOBAL_POSITION_INT', 'ATTITUDE'], blocking=True)
        
        # We need both position and attitude to calculate the vector
        # This is a simplified loop; in production, you'd cache the values.
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            current_alt = msg.relative_alt / 1000.0
            
            # If we are in the "Landing" phase (approx coords match target), start streaming
            # Or simply stream whenever we are close to the target lat/lon
            dist_to_pad_xy, _ = ll_to_local_m(TARGET_LAT, TARGET_LON, current_lat, current_lon)
            
            # Simple check: If within 20m of target, start streaming "Visual" data
            if abs(dist_to_pad_xy) < 20.0 and current_alt > 0.5:
                # We need Yaw, so we fetch the latest attitude
                att = m.recv_match(type='ATTITUDE', blocking=True)
                
                # Send the fake target data
                d, ax, ay = send_synthetic_target(m, current_lat, current_lon, current_alt, att.yaw)
                
                if int(time.time()*10)%5 == 0:
                    print(f"PLND Active: Dist={d:.1f}m AngX={ax:.1f} AngY={ay:.1f}")

if __name__ == "__main__":
    main()