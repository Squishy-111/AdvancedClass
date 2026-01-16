from pymavlink import mavutil
import time

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

# Ex: view all messages
# while True:
#    msg = the_connection.recv_match(blocking=True)
#    print(msg)

# Build a list of mission items in memory (as simple dicts)
mission_items = [
    # seq 0: WAYPOINT
    {
        # For some reason, waypoint 0 is never seen by the drone. This acts as a placeholder
        'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 
        'command': mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        'current': 1,
        'autocontinue': 1,
        'param1': 0, 'param2': 0, 'param3': 0, 'param4': 0,
        'x': int(0),                 # same as home
        'y': int(0),                 # same as home
        'z': 10                             # takeoff altitude
    },
    # se1 1: TAKEOFF
    {
        'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        'command': mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        'current': 1,
        'autocontinue': 1,
        'param1': 0, 'param2': 0, 'param3': 0, 'param4': 0,
        'x': int(0),                 # same as home
        'y': int(0),                 # same as home
        'z': 20                             # takeoff altitude
    },
    # seq 2: WAYPOINT
    {
        'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        'current': 0,
        'autocontinue': 1,
        'param1': 0, 'param2': 0, 'param3': 0, 'param4': 0,
        'x': int(53.5269 * 1e7),
        'y': int(-113.5256 * 1e7),
        'z': 20
    },
    # seq 3: WAYPOINT
    {
        'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        'current': 0,
        'autocontinue': 1,
        'param1': 0, 'param2': 0, 'param3': 0, 'param4': 0,
        'x': int(53.5271 * 1e7),
        'y': int(-113.5260 * 1e7),
        'z': 20
    },
    # seq 4: Landing
    {
        'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        'command': mavutil.mavlink.MAV_CMD_NAV_LAND,
        'current': 0,
        'autocontinue': 1,
        'param1': 0, 'param2': 0, 'param3': 0, 'param4': 0,
        'x': int(53.5271 * 1e7),
        'y': int(-113.5254 * 1e7),
        'z': 0
    }
]

# clear any existing mission
the_connection.mav.mission_clear_all_send(the_connection.target_system, the_connection.target_component)
time.sleep(0.2)

# tell vehicle how many items we will send
count = len(mission_items)
# Use the 3-argument form (system, component, count)
the_connection.mav.mission_count_send(the_connection.target_system, the_connection.target_component, count)

# respond to requests for mission items
sent = 0
while True:
    # wait for MISSION_REQUEST_INT or MISSION_REQUEST
    msg = the_connection.recv_match(type=['MISSION_REQUEST_INT','MISSION_REQUEST'], blocking=True, timeout=10)
    if not msg:
        print("Timed out waiting for mission request. Aborting.")
        break

    seq = msg.seq
    print("Mission request for seq", seq)

    if seq < 0 or seq >= count:
        print("Invalid seq requested:", seq)
        break

    item = mission_items[seq]

    # Send as mission_item_int (preferred for int frame)
    the_connection.mav.mission_item_int_send(
        the_connection.target_system,
        the_connection.target_component,
        seq,
        item['frame'],
        item['command'],
        item['current'],
        item['autocontinue'],
        item['param1'],
        item['param2'],
        item['param3'],
        item['param4'],
        item['x'],
        item['y'],
        int(item['z'])
    )
    print(f"Sent seq {seq}")
    sent += 1

    if sent >= count:
        # Wait for final ACK
        ack = the_connection.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
        if ack:
            print("Mission upload ACK:", ack)
            if getattr(ack, 'type', None) == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print("Mission uploaded successfully.")
            else:
                print("Mission upload rejected, type:", getattr(ack, 'type', None))
        else:
            print("No MISSION_ACK received.")
        break


# # Arm the drone
# the_connection.set_mode('STABILIZE') # cannot arm in automode
# the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
#                                      mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
# the_connection.set_mode('AUTO')
# 
# the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
#                                      mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)

