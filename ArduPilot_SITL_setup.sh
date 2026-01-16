sudo apt-get update
sudo apt-get install -y git python3 python3-pip python3-venv \
    build-essential libxml2-dev libxslt1-dev python3-dev

# ArduPilot & tools
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive

# install sim tools
cd Tools/autotest
pip3 install -r requirements.txt
