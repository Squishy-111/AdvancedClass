from pymavlink import mavutil
from webots_landing import LandingPadTracker
from webots_camera import CameraClient
import time
import cv2
import math


#This script assumes drone is facing completely north
#makes drone go near the center of the white square
#Then detects the white squares center
#Then detects how far drone is away from center then centers itself over white square
#then finally it lands directly on center of white square

PORT = 14551
the_connection = mavutil.mavlink_connection(f'udpin:0.0.0.0:{PORT}')

#grabbed this function to set mode from UofA mission script
def set_mode(name):
    mode_id = the_connection.mode_mapping()[name]
    the_connection.mav.set_mode_send(
        the_connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

#------------------------------------------------------------------
#this function is for sending waypoints local to drone's initial postion
def send_local_ned_target(x, y, z):
    
    type_mask = int(0b110111111000)

    the_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            int(time.time() * 1000) & 0xFFFFFFFF,
            the_connection.target_system,
            the_connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            x, y, z,
            0, 0, 0, # vx, vy, vz (Ignored)
            0, 0, 0, # ax, ay, az (Ignored)
            0, 0   # yaw, yaw_rate (yaw ignored by your mask)
        )
    )


#converts center of detectec ladning zone in pixels to (x,y) distance away in metres as seen from drone above
def get_distance_in_meters(pixel_x, pixel_y, altitude):

    #Calibrated directly for Webots camera
    fov_rad = 0.785
    img_width = 640
    img_height = 480

    #Calculate the "Ground Resolution" at this altitude
    visible_width_m = 2 * altitude * math.tan(fov_rad / 2)
    
    # Calculate scaling factor
    m_per_px = visible_width_m / img_width

    #Center the Coordinates
    center_x = img_width / 2
    center_y = img_height / 2
    
    offset_x_px = pixel_x - center_x
    offset_y_px = pixel_y - center_y

    #Convert to Meters (Camera Frame)
    cam_x_m = offset_x_px * m_per_px
    cam_y_m = offset_y_px * m_per_px
    
    #fixed coordinates from image converted to relative to axis of Arducopter
    body_y = cam_x_m

    body_x = -cam_y_m 

    return body_x, body_y

print('waiting for heartbeat')
the_connection.wait_heartbeat()
print('Heartbeat from system (system %u component %u)' % (the_connection.target_system, the_connection.target_component))

set_mode('GUIDED')

# arm
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)

# acknowledgement message
msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)

#waits until SITL says arduCopter at 10m before moving onto next command
while True:
  takeoff_msg = the_connection.recv_match(type="LOCAL_POSITION_NED", blocking=True)

  if takeoff_msg.z <= -10:
    break
  
#sets arduCopter to go to near landing zone (10,0,0) from existing gps coordinates
target_x, target_y, target_z = 9.5, 0, -10

while True:
    send_local_ned_target(target_x, target_y, target_z)

    msg = the_connection.recv_match(type="LOCAL_POSITION_NED", blocking=True)

    #checks if drone is near and SITL is sending confirmation data
    if msg and abs(msg.x - target_x) < 0.1 and abs(msg.z - target_z) < 0.1:
        print('target reached')
        current_x = msg.x
        current_y = msg.y
        break

    time.sleep(0.1)

#start looking for landing zone
tracker = LandingPadTracker()
webots_camera = CameraClient(tcp_ip='127.0.0.1', tcp_port=5599)

while True:
    #displays image of arduCopter
    webots_img = webots_camera.get_frame()
    found, target_center = tracker.process(webots_img)

    if found:
       time.sleep(3)#added so people can see the red bounding box appear before windows close
       pixel_x, pixel_y = target_center
       cv2.destroyAllWindows()
       break

#get x and y distance from arduCopter
offset_x, offset_y = get_distance_in_meters(pixel_x, pixel_y, 10)

#get the new gps target_x and target_y; target_z doesn't change.
target_x = offset_x + current_x
target_y = offset_y + current_y

#make the arduCopter center around white square
while True:
    send_local_ned_target(target_x, target_y, target_z)

    msg = the_connection.recv_match(type="LOCAL_POSITION_NED", blocking=True)

    #checks if drone is near and SITL is sending confirmation data
    if msg and abs(msg.x - target_x) < 0.1 and abs(msg.z - target_z) < 0.1:
        print('target reached')
        break

    time.sleep(0.1)

#land straight down
print("Initiating Vertical Landing...")
the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0, 0, 0, 0, 0, 
    0, 0, 0 
)

while True:
    #Monitor the heartbeat
    msg = the_connection.recv_match(type='HEARTBEAT', blocking=True)
    
    # Check if the Arducopter disarmed
    if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        print("Landed and Disarmed!")
        break
    
    time.sleep(0.5)
