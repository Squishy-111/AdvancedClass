# MAC GAZEBO SETUP
## Step 1: Have functinonal SITL
Ardupilot's gazebo flow expects you to be able to run sim_vehicle.py (this is the SITL script from ArduPilot). Most club members have this already, but if you're new then have SITL functional before proceeding.
## Step 2: Install Gazebo (Harmonic)
run: 
 
    brew tap osrf/simulation
    brew install gz-harmonic

Then verify install by opening a NEW terminal and running:

    gz sim -s -v4 -r shapes.sdf

And then open ANOTHER NEW terminal and run:

    gz sim -g -v4

The first terminal is running the server / physics, and the second opens the GUI.



## Step 3: Install all the Ardupilot Gazebo dependencies
run: 

    brew update
    brew install rapidjson
    brew install opencv gstreamer

Note that you may see error messages after this step. I had errors:
1) Error: The `brew link` step did not complete successfully
2) Error: Could not symlink include/QtDeviceDiscoverySupport/6.9.2/QtDeviceDiscoverySupport/private/qdevicediscovery_dummy_p.h

This is because OpenCV depends on qtbase but I already had qtbase installed and linked. Homebrew stopped to avoid overwriting files. To resolve, I then ran:

    brew unlink qt
    brew link qtbase
    brew install opencv

and then:

    brew install gstreamer



## Step 4: Clone and build the plugin

run:

    mkdir -p ~/gz_ws/src && cd ~/gz_ws/src
    git clone https://github.com/ArduPilot/ardupilot_gazebo
    cd ardupilot_gazebo
    export GZ_VERSION=harmonic
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
    make -j4

Again, this gave me some errors. If you saw lots of errors like I did, you can try running

    brew install qt@5
 
and then:

    export CMAKE_PREFIX_PATH="$(brew --prefix qt@5):$CMAKE_PREFIX_PATH"
    export Qt5_DIR="$(brew --prefix qt@5)/lib/cmake/Qt5"

and then a clean rebuild with:

    cd ~/gz_ws/src/ardupilot_gazebo
    rm -rf build
    mkdir build && cd build

    export GZ_VERSION=harmonic
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
    make -j4

## Step 5: Test

Unless you want to paste it into every new terminal you use to run Gazebo, you should add 

    export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"
    export GZ_SIM_RESOURCE_PATH="$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH"

to the bottom of your ~/.zshrc, and then reload the shell. 

## Step 6: run Gazebo + SITL
You will need 3 terminals. 

TERMINAL 1: Gazebo server
Run:

    cd ~/gz_ws/src/ardupilot_gazebo
    gz sim -s -v4 -r worlds/iris_runway.sdf

TERMINAL 2: GUI

Run:     

    gz sim -g -v4

TERMINAL 3: SITL

Run:

    sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map

Then, in terminal 3, run:

    mode guided
    arm throttle
    takeoff 3

if you see the quadcopter on the Gazebo GUI take off to ~3m and hover, then everything it as it should be






# ADVANCED GAZEBO SIM:
Once Gazebo is all set up, we can begin to actually do some meaninful simulation. Below is an example of a good simulation, but note that many steps are omitted due to the unque set up and issues I had. There is virtually no chance that if you have only completed the steps above that this will work for you. However, with some AI assistance and this setup guide as a reference for the AI, anyone should be able to get this working in ~1-3 hours.

## TERMINAL 1 (SERVER):
run: 

    cd ~/gz_ws/src/ardupilot_gazebo
    gz sim -s -v4 -r gimbal.sdf


## TERMINAL 2 (GUI):
run:

     gz sim -g -v4


## TERMINAL 3 (CAMERA STREAMING): 
Wait around 1 minute before running:

    gz topic -t /world/gimbal/model/mount/model/gimbal/link/pitch_link/sensor/camera/image/enable_streaming \
  -m gz.msgs.Boolean -p "data: 1"

this will enable streaming. Then, pitch gimble down so that camera is down facing:

    gz topic -t /gimbal/cmd_pitch -m gz.msgs.Double -p "data: 1.5708"

then, run:

    ffmpeg -protocol_whitelist file,udp,rtp \
  -i gazebo5600.sdp \
  -f mjpeg -q:v 5 -r 30 \
  -listen 1 http://127.0.0.1:8090/feed.mjpg


## TERMINAL 4:
For the down facing cameras POV, run:

    python ./view_http_mjpeg.py 

this script is in this repo.
