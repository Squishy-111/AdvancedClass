### Rasberry Pi 5 Setup Documentation

Making this documentation so future members of AeroDesign can put together one if we need another one next year

Using the RasTech Rasberry Pi 5 Starter Kit(8 GB ram version)

## Physical Setup

# mounting
1) I took the bottom half off of the Rpi Case  with my hands
2) I made sure I had the the Rpi 5 unboxed, small scredwriver, four screws and black power button ready
3) began installing the Rpi 5 in case by putting the black power button in the hole opposite to the HDMI ports(big end facing inwards), and fit the Rpi 5 in the bottom half of the case making the power button stay in place.
4) I then screwed in the the four screws in each yellow corner hole of the Rpi 5 mounting it to the case using the screwdriver

# active cooler installiation
1) I gathered the active cooler, four thermal pads and two push pins and previously mounted Rpi 5
2) I put the thermal pads on one at a time following the diagram given in the kit, making sure to remove the plastic on both sides
3) put the active cooler on the board and removed the cap for the connector in top right of board and connected the wires of the board into the Rpi 5
3) I then pushed the pins in the corners to mount the cooler to the board, making sure that the holes of the cooler aligned with the holes in the pi( required a bit of effor to actually push them in)
4) Then put the top half off the case back on careful not to split the wires

## online set up

hostname: AeroDesign-RPi5
username: aerodesign
password: aeroclub1234

# SD card initialization

1) downloaded rasberry Pi imager on my computer from rasberry pi website
2) took the back off my usb stick and inserted the SD card then inserted stick into my computer
3) ran the rasberry pi imager and started following the steps
4) added hostname above, added username and password above, enabled SSH using password authentication
5) Removed the usb stick once everything is finished and put the SD card into the pi

# SSH connection (using VS code)

1) download this extension in VS code: SSH - Remote
2) press the two arrows facing eachother in bottom left corner of VS code
3) press connect to host and enter this command in: ssh aerodesign@AeroDesign-RPi5.local
4) for the config option just press the first option given
5) now press the arrows facing eachother and click connect to host and enter the password above whenever it asks you, I also put my remote connection platform to linux
6) now just open the terminal in vs code and start making commands, I cloned our repo already in the desktop folder

# installing dependencies and wifi

I ran these commands:
sudo apt install python3-numpy
sudo apt install python3-opencv
sudo nmcli con add type wifi con-name "UWS" ssid "UWS"
pip3 install pymavlink mavproxy --break-system-packages

## Closing remarks

Everything is set up now but if have any questions about the process just shoot me a message on teams (Cameron Rozendaal)
