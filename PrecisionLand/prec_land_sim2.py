#!/usr/bin/env python3
"""
All-in-one Precision Landing simulator for ArduCopter SITL.
(Corrected to pre-position vehicle over the pad before LAND)
"""

import math
import time
import threading
from pymavlink import mavutil

MAVLINK_URL = "udpin:0.0.0.0:14551"
TAKEOFF_ALT = 30.0      # m
LT_RATE_HZ  = 10.0      # Hz
MAX_FOV_DEG = 20.0      # clamp angles to +- this



# New function for commanding position in GUIDED mode
# Updated function with explicit integer casting for safety
def send_position_target(m, lat, lon, alt_m):
    """Sends SET_POSITION_TARGET_GLOBAL_INT to command the vehicle to fly to a location."""
    
    # Type Mask: Use position (Lat/Lon/Alt) but ignore velocity, acceleration, and yaw/yaw_rate
    # 0b0000111111111000 = 0x0F F8 (Decimal 4088)
    type_mask = 0b0000111111111000 
    
    # CRITICAL: Ensure we use the correct integer representation for the required INT32 fields
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)

    m.mav.set_position_target_global_int_send(
        int(time.time() * 1000) & 0xFFFFFFFF, # time_boot_ms: ensure it's within uint32 limits
        m.target_system,                      # target_system
        m.target_component,                   # target_component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame (Alt is relative to home)
        type_mask,                            # type_mask 
        lat_int,                              # lat_int (p1)
        lon_int,                              # lon_int (p2)
        alt_m,                                # alt (p3)
        0, 0, 0,                              # vx, vy, vz (p4, p5, p6 - ignored by mask)
        0, 0, 0,                              # afx, afy, afz (p7, p8, p9 - ignored by mask)
        0.0,                                  # yaw (p10 - required, but ignored by mask)
        0.0                                   # yaw_rate (p11 - required)
    )
    print(f"Sent SET_POSITION_TARGET_GLOBAL_INT to Lat={lat:.7f}, Lon={lon:.7f}, Alt={alt_m}m.")
    return True

def connect_mavlink():
    print(f"Connecting to {MAVLINK_URL} ...")
    m = mavutil.mavlink_connection(MAVLINK_URL)
    m.wait_heartbeat()
    print("Heartbeat from system %u component %u" % (m.target_system, m.target_component))
    return m


def _normalize_param_id(pid):
    if isinstance(pid, bytes):
        return pid.decode(errors="ignore").rstrip("\x00")
    return str(pid).rstrip("\x00")


def set_param(m, name, value, ptype=None, wait_echo=True, timeout=2.0):
    if ptype is None:
        ptype = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    m.mav.param_set_send(
        m.target_system, m.target_component,
        name.encode("utf-8"), float(value), ptype
    )
    if not wait_echo:
        return
    t0 = time.time()
    while time.time() - t0 < timeout:
        pv = m.recv_match(type="PARAM_VALUE", blocking=False)
        if not pv:
            time.sleep(0.02)
            continue
        pid = _normalize_param_id(pv.param_id)
        if pid == name:
            print(f"{name} -> {pv.param_value}")
            return
    print(f"PARAM_VALUE echo timeout for {name}")


def set_mode(m, name):
    mode_id = m.mode_mapping()[name]
    m.mav.set_mode_send(
        m.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )


def cmd_long(m, cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        cmd, 0, p1, p2, p3, p4, p5, p6, p7
    )


def wait_position(m, timeout=20.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(blocking=False)
        if not msg:
            time.sleep(0.02)
            continue
        t = msg.get_type()
        if t == "GLOBAL_POSITION_INT":
            return True
        if t == "GPS_RAW_INT" and getattr(msg, "fix_type", 0) >= 3:
            return True
    return False


def wait_altitude(m, alt_m, timeout=30.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        gpi = m.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if gpi:
            current_alt = gpi.relative_alt / 1000.0
            if current_alt >= alt_m * 0.95:  # Wait until it's near the target alt
                print(f"Reached {current_alt:.1f}m. Continuing...")
                return True
        time.sleep(0.1)
    print(f"Timeout waiting for altitude to reach {alt_m}m.")
    return False

# New function to wait for the vehicle to reach the target GPS location
def wait_location(m, target_lat, target_lon, alt_m, tolerance_m=3.0, timeout=40.0):
    t0 = time.time()
    R = 6378137.0
    print(f"Waiting for vehicle to reach pad location (tolerance {tolerance_m}m)...")
    while time.time() - t0 < timeout:
        gpi = m.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if gpi:
            vlat = gpi.lat / 1e7
            vlon = gpi.lon / 1e7
            
            # Calculate distance using rough ECEF conversion for small distances
            dlat = math.radians(vlat - target_lat)
            dlon = math.radians(vlon - target_lon)
            x_east = R * dlon * math.cos(math.radians((vlat + target_lat) / 2.0))
            y_north = R * dlat
            distance = math.sqrt(x_east**2 + y_north**2)
            
            current_alt = gpi.relative_alt / 1000.0

            if distance < tolerance_m and abs(current_alt - alt_m) < 5.0:
                print(f"Vehicle is hovering near target ({distance:.1f}m error).")
                return True
            
            # Show progress occasionally
            if int(time.time() * 2) % 2 == 0:
                 print(f"Distance to pad: {distance:.1f} m, Altitude: {current_alt:.1f} m")
        
        time.sleep(0.5)
    print("Timeout waiting for vehicle to reach target location.")
    return False


def ll_to_local_m(lat, lon, lat0, lon0):
    R = 6378137.0
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    x_east = R * dlon * math.cos(math.radians((lat + lat0) / 2.0))
    y_north = R * dlat
    return x_east, y_north


def local_m_to_ll(x_east, y_north, lat0, lon0):
    R = 6378137.0
    dlat = y_north / R
    dlon = x_east / (R * math.cos(math.radians(lat0)))
    return lat0 + math.degrees(dlat), lon0 + math.degrees(dlon)


class LTStreamer:
    # ... (LTStreamer class is unchanged as the problem is mode engagement, not streaming logic)
    """
    Streams LANDING_TARGET based on pad GPS, but:
      - only when airborne (relative_alt > 0.3 m)
      - only when in LAND mode
      - angles clamped to +/- MAX_FOV_DEG using a soft “10 m => 20°” mapping
      - distance fixed ~2 m (plausible with typical rangefinder config)
    """

    def __init__(self, m, pad_lat, pad_lon, rate_hz=10.0):
        self.m = m
        self.pad_lat = pad_lat
        self.pad_lon = pad_lon
        self.rate = rate_hz
        self.stop = False
        self.last_gpi = None
        self.last_att = None

    def _update_cache(self):
        # We need to read more often since the main thread is now very busy
        for _ in range(5): 
             msg = self.m.recv_match(
                type=["GLOBAL_POSITION_INT", "ATTITUDE", "HEARTBEAT"],
                blocking=False
            )
             if msg:
                t = msg.get_type()
                if t == "GLOBAL_POSITION_INT":
                    self.last_gpi = msg
                elif t == "ATTITUDE":
                    self.last_att = msg

    def _send_target(self):
        gpi = self.last_gpi
        att = self.last_att
        if not (gpi and att):
            return

        # Only send when definitely off the ground a bit
        z = gpi.relative_alt / 1000.0  # m AGL
        if z < 0.3:
            return

        # Only send while in LAND mode (mimic camera used just for landing)
        mode = self.m.flightmode
        if mode != "LAND":
            return

        vlat = gpi.lat / 1e7
        vlon = gpi.lon / 1e7

        # pad - veh in local EN
        ex, ny = ll_to_local_m(self.pad_lat, self.pad_lon, vlat, vlon)
        yaw = att.yaw

        # EN -> body FRD
        bx = math.cos(yaw) * ny + math.sin(yaw) * ex   # forward
        by = -math.sin(yaw) * ny + math.cos(yaw) * ex  # right

        # Map horizontal offsets to modest angles
        max_ang = math.radians(MAX_FOV_DEG)
        scale_m = 10.0  # meters horizontal offset that maps to max_ang

        nx = max(min(by / scale_m, 1.0), -1.0)   # right
        ny_b = max(min(bx / scale_m, 1.0), -1.0) # forward
        angle_x = nx * max_ang
        angle_y = ny_b * max_ang

        # Fixed, plausible distance (2 m)
        distance = 2.0

        # occasional debug
        if int(time.time() * 10) % 10 == 0:
            # Added distance_to_pad for clearer debugging
            distance_to_pad = math.sqrt(ex**2 + ny**2)
            print(
                f"PLND feed: mode={mode}, z={z:4.1f} m, "
                f"dist={distance_to_pad:5.1f} m, " 
                f"bx={bx:5.1f} m, by={by:5.1f} m, "
                f"ax={math.degrees(angle_x):5.1f}°, "
                f"ay={math.degrees(angle_y):5.1f}°"
            )

        msg = self.m.mav.landing_target_encode(
            int(time.time() * 1000) & 0xFFFFFFFF,  # time_boot_ms
            0,                                    # target_num
            mavutil.mavlink.MAV_FRAME_BODY_FRD,   # frame
            float(angle_x),
            float(angle_y),
            float(distance),
            0.05,                                 # size_x dummy
            0.05,                                 # size_y dummy
        )
        self.m.mav.send(msg)

    def run(self):
        dt = 1.0 / self.rate
        while not self.stop:
            self._update_cache()
            self._send_target()
            time.sleep(dt)

   

def main():
    m = connect_mavlink()

    # --- PLND params (match your working example more closely) ---
    print("Setting PLND / LAND params...")
    set_param(m, "PLND_ENABLED",   1, ptype=mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    set_param(m, "PLND_TYPE",      1, ptype=mavutil.mavlink.MAV_PARAM_TYPE_INT32)  # MAVLink
    set_param(m, "PLND_EST_TYPE",  0, ptype=mavutil.mavlink.MAV_PARAM_TYPE_INT32)  # no estimator
    set_param(m, "PLND_ORIENT",    0, ptype=mavutil.mavlink.MAV_PARAM_TYPE_INT32)  # down
    set_param(m, "PLND_YAW_ALIGN", 0, ptype=mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    set_param(m, "PLND_ALT_MAX",  50)
    set_param(m, "PLND_ALT_MIN",   0)
    set_param(m, "LAND_REPOSITION", 1, ptype=mavutil.mavlink.MAV_PARAM_TYPE_INT32)

    # Note: PLND_BUS_MAX is intentionally omitted as it does not exist.

    print("Waiting for position...")
    if not wait_position(m):
        raise RuntimeError("No position estimate yet")

    # Get home location
    home_gpi = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
    if not home_gpi:
        raise RuntimeError("Failed to get home GLOBAL_POSITION_INT")

    home_lat = home_gpi.lat / 1e7
    home_lon = home_gpi.lon / 1e7
    print(f"Home at lat={home_lat:.7f}, lon={home_lon:.7f}")

    # Pad ~10 m east & 10 m north
    pad_lat, pad_lon = local_m_to_ll(10.0, 10.0, home_lat, home_lon)
    print(f"Pad at  lat={pad_lat:.7f}, lon={pad_lon:.7f}")

    # GUIDED -> arm -> takeoff
    print("Switching to GUIDED...")
    set_mode(m, "GUIDED")
    time.sleep(1.0)

    print("Arming...")
    cmd_long(m, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    time.sleep(2.0)

    print(f"Takeoff to {TAKEOFF_ALT} m...")
    cmd_long(
        m,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0,
        0, 0, TAKEOFF_ALT
    )

    # Wait until it reaches takeoff altitude
    wait_altitude(m, TAKEOFF_ALT)

    # --- NEW CRITICAL STEP: FLY TO PAD LOCATION ---
    print(f"Flying to pad location: lat={pad_lat:.7f}, lon={pad_lon:.7f}...")
    print(f"Flying to pad location: lat={pad_lat:.7f}, lon={pad_lon:.7f}...")
    
    # Use the new function to send and wait for acceptance
    if not send_position_target(m, pad_lat, pad_lon, TAKEOFF_ALT):
        raise RuntimeError("Failed to send SET_POSITION_TARGET_GLOBAL_INT.")

    # Wait for the vehicle to arrive and hover above the pad
    if not wait_location(m, pad_lat, pad_lon, TAKEOFF_ALT):
        # We raise a RuntimeError here if it still fails to reach the pad
        raise RuntimeError("Failed to reach target location before timeout.")

    # Start LANDING_TARGET streamer
    streamer = LTStreamer(m, pad_lat, pad_lon, rate_hz=LT_RATE_HZ)
    th = threading.Thread(target=streamer.run, daemon=True)
    th.start()
    
    # Give the streamer a moment to send data while hovering
    time.sleep(1.0) 

    print("Switching to LAND (PLND should engage now)...")
    set_mode(m, "LAND")

    t0 = time.time()
    while time.time() - t0 < 60.0:
        msg = m.recv_match(type=["STATUSTEXT", "GLOBAL_POSITION_INT"], blocking=False)
        if msg:
            print(msg)
        time.sleep(0.05)

    streamer.stop = True
    print("Done.")


if __name__ == "__main__":
    main()