#!/usr/bin/env python3
# Precision Landing (AprilTag-style) simulator for ArduPilot SITL

from pymavlink import mavutil
import math, time, threading

PY_PORT = 14551
TAKEOFF_ALT = 30

PAD_LAT = 53.523219 + 0.00001
PAD_LON = -113.526319 + 0.00001

def connect(port):
    # Open a MAVLink connection that listens on UDP port specified.
    # udpin means bind a local UDP socket and wait for packets (passive listener).
    # 0.0.0.0 lets it receive from any interface (localhost, Docker, etc)
    m = mavutil.mavlink_connection(f"udpin:0.0.0.0:{port}")
    print("Waiting for heartbeat…")

    # Block until the first heartbeat arrives from the vehicle (via SITL/MAVProxy router).
    # This also populates m.target_system and m.target_component so later commands
    # are addressed to the correct system/component ID's
    m.wait_heartbeat()
    print("Connected to sys", m.target_system, "comp", m.target_component)

    # Return a ready to use connection. From now on you can send modes, commands,
    # and missions through m (ex: m.set_mode_send)
    return m


def set_mode(master, name):
    # mode_mapping() asks the FC for a dictionary of flight mode ID's the vehicle understands
    # ex: {'STABILIZE': 0, 'LOITER': 5}. We retreive the mode ID we want to set by name
    mode_id = master.mode_mapping()[name]

    # Set the mode to whatever mode id we retreived from the previous step. 
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

def cmd_long(master, cmd, p1=0,p2=0,p3=0,p4=0,p5=0,p6=0,p7=0):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        cmd, 0, p1,p2,p3,p4,p5,p6,p7
    )

def _normalize_param_id(pid):
    # pid may be bytes or str depending on pymavlink version
    if isinstance(pid, bytes):
        return pid.decode(errors="ignore").rstrip("\x00")
    return str(pid).rstrip("\x00")

def set_param(master, name, value, ptype=None, wait_echo=True, timeout=2.0):
    """
    Send a PARAM_SET and (optionally) wait for the matching PARAM_VALUE echo.
    ptype: None => float (REAL32). Use MAV_PARAM_TYPE_INT32 for ints if you want.
    """
    if ptype is None:
        # ArduPilot stores most params as floats; ints also work.
        ptype = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    master.mav.param_set_send(master.target_system, master.target_component,
                              name.encode("utf-8"), float(value), ptype)
    if not wait_echo:
        return
    t0 = time.time()
    while time.time() - t0 < timeout:
        pv = master.recv_match(type="PARAM_VALUE", blocking=False)
        if pv:
            pid = _normalize_param_id(pv.param_id)
            if pid == name:
                # print(f"{name} -> {pv.param_value}")
                return
        time.sleep(0.02)
    # No echo?
    # print(f"PARAM_VALUE echo timeout for {name}")


# Function to make sure the our flight controller knows where it is before we start doing any flying
def wait_position(master, timeout=20):
    t0 = time.time()
    while time.time() - t0 < timeout:
        m = master.recv_match(blocking=False)
        if not m:
            time.sleep(0.02); continue
        t = m.get_type()
        if t == "GLOBAL_POSITION_INT": return True
        if t == "GPS_RAW_INT" and getattr(m, "fix_type", 0) >= 3: return True
    return False

def ll_to_local_m(lat, lon, lat0, lon0):
    R = 6378137.0
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    x_east  = R * dlon * math.cos(math.radians((lat+lat0)/2.0))
    y_north = R * dlat
    return x_east, y_north

class LTStreamer:
    def __init__(self, master, pad_lat, pad_lon, rate_hz=20):
        self.m = master
        self.pad_lat = pad_lat
        self.pad_lon = pad_lon
        self.rate = rate_hz
        self.stop = False
        self.last_att = None
        self.last_gpi = None

    def _update_cache(self):
        msg = self.m.recv_match(type=["GLOBAL_POSITION_INT","ATTITUDE"], blocking=False)
        if msg:
            if msg.get_type() == "GLOBAL_POSITION_INT":
                self.last_gpi = msg
            elif msg.get_type() == "ATTITUDE":
                self.last_att = msg

    def _send_target(self):
        gpi, att = self.last_gpi, self.last_att
        if not (gpi and att):
            #print("FAILED!!!!!")
            return

        # print("PASSED!!!!!")
        vlat = gpi.lat / 1e7
        vlon = gpi.lon / 1e7
        z = max(gpi.relative_alt / 1000.0, 0.1)  # m AGL

        ex, ny = ll_to_local_m(self.pad_lat, self.pad_lon, vlat, vlon)   # pad - veh (E,N)
        print(f"Target is {ex} meters right and {ny} meters north of the drone")
        yaw = att.yaw  # rad
        # EN -> body FRD rotation (approx local-frame)
        bx =  math.cos(yaw)*ny + math.sin(yaw)*ex     # forward
        by = -math.sin(yaw)*ny + math.cos(yaw)*ex     # right

        angle_x = math.atan2(by, z)  # +right
        angle_y = math.atan2(bx, z)  # +forward
        dist = math.sqrt(bx*bx + by*by + z*z)

          # debug (e.g. print every 10th call)
        if int(time.time() * 10) % 10 == 0:
            print(f"bx={bx:.1f} by={by:.1f} z={z:.1f} ax={angle_x:.3f} ay={angle_y:.3f}")

        # time in *microseconds* (int), not ms
        time_us = int(time.time() * 1e6)

        # Optional 3D position & orientation fields (we don't use them here)
        x = y = z = 0.0
        q = [1.0, 0.0, 0.0, 0.0]  # unit quaternion placeholder

        # Integers that must be ints in this dialect:
        target_num = 0
        frame = mavutil.mavlink.MAV_FRAME_BODY_FRD
        lt_type = 0            # 0 = generic/unspecified
        position_valid = 1     # 1 = we consider this measurement valid

        self.m.mav.landing_target_send(
            time_us,           # uint64
            target_num,        # uint8
            frame,             # uint8
            float(angle_x),    # float
            float(angle_y),    # float
            float(dist),       # float
            0.0, 0.0,          # size_x, size_y (unused)
            x, y, z,           # optional position (unused)
            q,                 # 4-element quaternion
            lt_type,           # uint8
            position_valid     # uint8
        )                                # sensor type (unused)
        

    def run(self):
        dt = 1.0 / self.rate
        while not self.stop:
            self._update_cache()
            self._send_target()
            time.sleep(dt)

def main():
    m = connect(PY_PORT) # establish a connection to mavlink

    # Precision Landing params (MAVLink backend). You can also go into QGC/MissionPlanner and set these manually but this assures theyre set before running
    set_param(m, "PLND_ENABLED", 1, ptype=mavutil.mavlink.MAV_PARAM_TYPE_INT32) 
    set_param(m, "PLND_TYPE",    1, ptype=mavutil.mavlink.MAV_PARAM_TYPE_INT32)  # 1 = MAVLink
    set_param(m, "PLND_ORIENT",  0, ptype=mavutil.mavlink.MAV_PARAM_TYPE_INT32)  # Down camera orientation
    set_param(m, "PLND_YAW_ALIGN", 0, ptype=mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    set_param(m, "PLND_ALT_MAX", 50)
    set_param(m, "PLND_ALT_MIN", 0)

    # Flight: GUIDED -> wait EKF -> arm -> takeoff. Supposed to reach 20m alt but it doesnt... likely due to the "sleep (5)" time limit on ascend command
    set_mode(m, "GUIDED")
    if not wait_position(m): raise RuntimeError("No position estimate yet")
    cmd_long(m, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1) # 1 = arm, 0 = disarm 
    time.sleep(1.0)
    cmd_long(m, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,0,0,0, 0,0, TAKEOFF_ALT)
    print(f"Taking off to {TAKEOFF_ALT} m…")

    # Start LANDING_TARGET stream aimed at the pad
    streamer = LTStreamer(m, PAD_LAT, PAD_LON, rate_hz=20)
    th = threading.Thread(target=streamer.run, daemon=True)
    th.start()

    time.sleep(5.0)  # Climb for 5 seconds
    set_mode(m, "LAND")
    print("LAND commanded — precision landing engaging…")

    t0 = time.time()
    while time.time() - t0 < 60:
        msg = m.recv_match(type=["STATUSTEXT","GLOBAL_POSITION_INT"], blocking=False)
        if msg:
            print(msg)
        time.sleep(0.05)

    streamer.stop = True
    print("Done.")

if __name__ == "__main__":
    main()
