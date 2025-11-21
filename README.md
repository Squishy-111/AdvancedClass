AdvancedClass demo, Oct 9 2025:

Install [Anaconda](https://www.anaconda.com/download) at your system-level for virtual environment.

- For Linux run `bash yolo_setup.sh` 

- Mac users may run the command `sh yolo_setup.sh` (someone with Mac test before merging)

- Windows users running WSL may run the command "./yolo_setup.sh" (still should test)

After running the command, activate conda's virtual environment with `conda activate yolo` and any further `python` and `pip` commands will run isolated from your system environment.

Running the scripts with `python` after this should produce an error that it can't find the serial device `cu.usbserial-DN04T9FH`. 

## Resolving Serial Port Connection Errors

### macOS

1. Ensure your **telemetry radio** or **flight controller** is plugged in via USB.
2. Open **Terminal** and run:
   ```bash
   ls /dev/tty.*
   ```
3. You should see output similar to:
   ```
   /dev/cu.usbserial-DN04T9FH
   ```
4. Use this result in your Python code:
   ```python
   open_serial("/dev/cu.usbserial-DN04T9FH", 57600)
   ```
   Replace the first argument with the device path shown on your system.

---

### Windows

1. Plug in your **telemetry radio** or **flight controller**.
2. Open **PowerShell** and run:
   ```powershell
   Get-WmiObject Win32_SerialPort | Select-Object DeviceID, Name, Description
   ```
3. Example output:
   ```
   DeviceID  Name                 Description
   --------  ----                 -----------
   COM3      USB-SERIAL CH340     USB-SERIAL CH340
   COM5      PX4 FMU V5           PX4 Autopilot
   ```
4. Use the correct `DeviceID` (COM port) for your telemetry radio:
   ```python
   open_serial("COM3", 57600)
   ```
   Replace `"COM3"` with whichever COM port appears in your system.

---

### At This Point
Your script should now connect successfully **if the transmitter is plugged into your USB port**.

---

### Important Note
**QGroundControl** automatically connects to the transmitter by default.  
Before running any Python scripts that use the same serial device, **close or disconnect QGroundControl** to avoid port conflicts.


# Run SITL with docker:
docker run -it --rm orthuk/ardupilot-sitl bash -lc '
mkdir -p ~/.config/ardupilot
# name=lat,lon,alt(m MSL),heading(deg)
printf "UofA=53.523219,-113.526319,676,0\n" > ~/.config/ardupilot/locations.txt
./Tools/autotest/sim_vehicle.py -v ArduCopter -L UofA \
  --out=udp:host.docker.internal:14550
'



docker run -it --rm orthuk/ardupilot-sitl bash -lc '
mkdir -p ~/.config/ardupilot
printf "UofA=53.523219,-113.526319,676,0\n" > ~/.config/ardupilot/locations.txt
./Tools/autotest/sim_vehicle.py -v ArduCopter -L UofA \
--out=udp:host.docker.internal:14550 \
--out=udp:host.docker.internal:14551


