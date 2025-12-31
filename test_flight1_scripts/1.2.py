from pymavlink import mavutil
import time

# Start a connection listening on a UDP port
PORT = 14551
the_connection = mavutil.mavlink_connection(f'udpin:0.0.0.0:{PORT}')


# picked up from uofa_mission.py
def set_mode(name):
    mode_id = the_connection.mode_mapping()[name]
    the_connection.mav.set_mode_send(
        the_connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
print("waiting for heartbeat")
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

set_mode("GUIDED")

# arm
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

# acknowledgement message
msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)


# setting altitude to 2 meters using takeoff methdo
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 2)

# acknowledgement message
msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)

takeoff_altitude = -2

# waiting until altitude is 2 meters
while True:
  msg = the_connection.recv_match(type="LOCAL_POSITION_NED", blocking=True)
  print(msg)

  if msg.z <= takeoff_altitude:
     break

print("Aircraft has reached cruising altitude")


# takeoff and fly towards first waypoint (10m up and 10m ahead)
the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000), 10, 0, -10, 0, 0, 0, 0, 0, 0, 0, 0))

# acknowledgement message
msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)


while True:
  msg = the_connection.recv_match(type="LOCAL_POSITION_NED", blocking=True)
  print(msg)

  if msg.z <= -9.8 and msg.x >= 9.8:
     break

print("Aircraft has reached first waypoint")


# fly towards second waypoint (10m right from current position)
the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000), 10, 10, -10, 0, 0, 0, 0, 0, 0, 0, 0))

# acknowledgement message
msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)


while True:
  msg = the_connection.recv_match(type="LOCAL_POSITION_NED", blocking=True)
  print(msg)

  if msg.z <= -9.8 and msg.x >= 9.8 and msg.y >= 9.8:
     break

print("Aircraft has reached second waypoint")


print("initiating landing")

# landing aircraft back down
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)

# acknowledgement message
msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)

while True:
  msg = the_connection.recv_match(type="LOCAL_POSITION_NED", blocking=True)
  print(msg)
  
  if msg.z >= -0.1:
     break
  
print("Aircraft has landed")