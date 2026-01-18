This documentation is for Windows Users using WSL Ubuntu

I am going into the steps on how to get the simulation working assuming you've already got SITL running fine

1. go to the webots website and download it for windows

2. get the run_webots_sitl.sh in the webots simulation folder in the advancedclass repository and add it to your ardupilot folder in linux where your other SITL baschript for missionplanner should be

3. Then go copy the script iris_landing_test.wbt in webots simulation folder and put it in the worlds folder with the other .wbt files: Linux\Ubuntu\home\YOUR_USER_NAME\ardupilot\libraries\SITL\examples\Webots_Python\worlds

4. Then once your file is in the folder run it from there

5. Once Webots is running go to the top bar and press tools -> preferences and change startup_mode to pause (VERY IMPORTANT)

6. close the webots world

6. Then go into the Ubuntu Terminal and run the following command: hostname -I

7. This gives you your wsl ip address, now copy it.

8. Now open up your webots world again it should be paused

9. The click the small arrow beside Iris on the left in webots, then click the small arrow beside controllerArgs. You should see the ip address under --sitl-address. click on the ip address then change it to the one you copied in the ubuntu terminal.

10. Now go click on the top left tab: file,  in webots and click save world and then close webots.

11. (HARD STEP, get AI to clarify) Now please go to the file webots_vehicle.py found in the below path
Linux\Ubuntu\home\YOUR_USER_NAME\ardupilot\libraries\SITL\examples\Webots_Python\controllers\ardupilot_vehicle_controller

12. go to the function _handle_controls and change the first lines of code in that function to look like this:

command_motors = command[:len(self._motors)]
        if -1 in command_motors:
            if self.robot.getTime() > 5.0: #make sure to use spaces when inputting and not tabs
                print(f"Warning: SITL provided {command.index(-1)} motors but model specifies {len(self._motors)} (I{self._instance})")

Basically just add the second if statement so it doesnt encounter any motor errors

13. Then go down to the function _handle_image_stream scroll down to where webots sends over the image stream and change the marked lines to whats shown below.

                    # get image
                    if isinstance(camera, Camera):
                        img = self.get_camera_image() #change this line to get_camera_image()instead of get_camera_gray_image()
                    elif isinstance(camera, RangeFinder):
                        img = self.get_rangefinder_image()

basically makes it so that it sends an RGB image over the TCP

14. now save the file and close it.

15. now go to the file iris in the path below:
Linux\Ubuntu\home\YOUR_USER_NAME\ardupilot\libraries\SITL\examples\Webots_Python\params

16. open the file and make sure these four lines are at the bottom:
SERVO1_FUNCTION 33
SERVO2_FUNCTION 34
SERVO3_FUNCTION 35
SERVO4_FUNCTION 36

17. if these lines arent there then add them, save the file, and close it.

18. now you should be ready to start the simulation

19. Now make sure you follow this order whenever your running anything in Webots
    1. open the webots world
    2. run the script run_webots_sitl.sh
    3. when the script in ubuntu says link 1 down or something similar, go to webots and press play
    4. now run your python script to give sitl commands

20. Now everything should be ready to go, I recommend testing webots_landing.py or one of the testflight scripts.

If you have any problems send me a message on teams, Cameron Rozendaal.

Once everything is working perfectly look into my scripts and maybe add different objects to webots and get experience with making different pymavlink scripts.



