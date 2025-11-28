# Installing and setting up the ArduPilot repo in WSL

1) Install WSL for windows. It should be as simple as running the following command in powershell and then restarting your computer: wsl --install
2) Search "ubuntu" in the start menu and open the application
3) Within the application, run these three commands (Press Y+Enter if asked to continue):
   - sudo apt-get update
   - sudo apt-get install git
   - sudo apt-get install gitk git-gui
4) Clone ArduPilot's repo by running the following command:
   - git clone "https://github.com/ArduPilot/ardupilot.git"
5) Enter the ArduPilot repository by entering the following command:
   - cd ardupilot
6) Run the following commands (this is a script that ArduPilot provides that will install some required packages):
   - Tools/environment_install/install-prereqs-ubuntu.sh -y
   - . ~/.profile
7) Exit Ubuntu, go into windows powershell, and type the following command:
   - wsl --shutdown
8) Reopen Ubuntu, if there's a path in brackets before your actual path (indicating a virtual environment), run the following command:
   - deactivate


# Setting up the SITL simulator:
1) Type the following command:
   - cd ardupilot
2) Run the following command to build SITL:
   - ./waf configure --board sitl && ./waf plane
   - if the output says you need to install a specific package, run the following command and then rebuild SITL using the command above:
      - sudo apt install python3-<package_name>
3) Run the following commands in order to correctly install MavProxy and run SITL:
   - sudo apt update && sudo apt upgrade -y
   - sudo apt install python3-pip python3-dev python3-setuptools python3-wheel -y
   - sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame
   - pip install --break-system-packages "numpy<2"
   - pip install --break-system-packages --force-reinstall opencv-python
   - pip install pymavlink MAVProxy --break-system-packages
   - echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc
   - . ~/.bashrc


# Running the SITL simulator
1) Within the ardupilot repo, move to the ArduCopter directory by running the following command:
   - cd ArduCopter
2) run the following command to run SITL:
   - ../Tools/autotest/sim_vehicle.py --map --console
   If you get an error saying "No module named 'future'", run this command and then try the above command again:
      - pip install future --break-system-packages


# Creating a bashscript within the ArduPilot repo to start SITL after setup
1) Go into the Advanced Class repository on github, go into the scripts directory where you will find the "run_sitl_win.sh"
2) Copy all of the code inside the directory
3) Within the ardupilot repo on WSL, create and enter into a bash script file using the following command (make sure you are in the root directory of the repo):
   - vim run_sitl_win.sh
4) Press i to go into insert mode and edit the script
5) Paste the code previously copied from the Advanced Class repository
6) Press Esc to exit out of insert mode
7) press :wq to save and quit the file
8) run the following command:     (NEW)
   - cd Tools/autotest          
9) run this command:              (NEW)
   - vim locations.txt
10) press i to edit file then paste this line at the bottom:  (NEW)
    - UofA=53.523219,-113.526319,0,90
11) press esc then type ':wq' to save and exit the file      (NEW)
12) now put in the two commands to get back to the root directory:   (NEW)
    - cd
    - cd ardupilot
13) Make the script executable by running the following command:
   - chmod +x run_sitl_win.sh
14) Run the file by running the following command from the root directory:
   - ./run_sitl_win.sh

### Reach out to me (Cameron) on teams for questions concerning the new changes or just help with the process (also make sure mission planner is only connected to udp 14550)

### Note: Setup steps can vary sometimes from machine to machine. If you're running into any troubles while using this documentation, please don't hesitate to reach out to me (Hassan) over Teams and hopefully we can figure it out together :)