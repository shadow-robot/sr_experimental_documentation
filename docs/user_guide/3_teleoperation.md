# Teleoperation system

## Hardware needed
### Laptop and Adapters

* Laptop 3XS with label PROD-DEMO-LAPTOP-2 + Laptop Charger
[Picture]
* UGREEN USB 3.0 to Ethernet RJ45 Adapter for Hand connection
￼
* Aukey USB-A to 3-Port USB 3.0 with Ethernet Adapter (model: CB-H15)

### CyberGlove
* Cyberglove + White gloves
[Picture]
                                                                                                                                                                          
* CyberGlove Power Switch + D-Sub type connector + USB Serial Adaptor

(Picture)

### HTC Vive
* VIVE tracker + Micro USB + Power Adapter
* VIVE controller + Micro USB + Power Adapter
* 2 VIVE lighthouses + 2 Power Cables
* VIVE headset + 3-in-1 Cable 
* Link Box + Power Cable                                                                                                                                  
* HDMI to HDMI cable
* USB to USB Cable
* 2 tripods for mounting the 2 lighthouses around the operator

### UR10 Robot Arm + Dexterous Hand + Demo Table
* UR10 with supported [firmware]()
* 2 x Ethernet Cable
* Arm Control Box
* Hand E Mounting Plate
* 8 x M6 12 mm screws
* 4 x M6 Long Screws
* Emergency Button
* Hand E
* Power Supply
* Demo Table

```eval_rst
.. image:: ../img/UR10_hand_E.jpeg
  :width: 400
  :align: center
```

## Checklist
### Equipment Check
* Check previous section for all the hardware needed
### Hand Firmware Check
Check the hand firmware version by executing those instructions:

1. Plug the Dexterous Hand necessary cables by following the instructions described [here](https://dexterous-hand.readthedocs.io/en/latest/user_guide/1_setting_up_the_hand.html#connecting-cables).
2. Start the laptop and when ready press CTRL+ALT+T to open a new terminal.
3. In the pop-up terminal run the driver by running the following command:
   ```sh
   roslaunch sr_ethercat_hand_config sr_rhand.launch
   ```
   ```eval_rst
   .. Note:: If the hand does not start correctly, you probably need to change the ethernet port. Contact someone from the Software Team.
   ```
4. In the terminal scroll up to the firmware version that gets printed in yellow and it should look like in the picture below:
5. Make sure the firmware version corresponds to the first row and second column of the [Firmware version and git commit hash table](https://shadowrobot.atlassian.net/wiki/spaces/HANDEG/pages/153124930/Firmware+version+and+git+commit+hash+table).
   ```eval_rst
   .. Note:: If it does not or you have any doubt about this procedure, please contact someone from the Software Team
   ```
### UR10 Firmware Check

1. Start the UR10 pendant by pressing the power button.
2. On the right top corner of the screen, click on the About button
3. Check the software version by comparing the number highlighted in the picture below with the list of UR10 Supported firmware
4. If you cannot find the number displayed in the screen in the list of [UR10 Supported firmware](https://shadowrobot.atlassian.net/wiki/spaces/SDSR/pages/371687449/UR10+Supported+firmware), please contact someone from the Software Team.

### Collision scene Check

The geometry scene for collision avoidance is by default the one in our Office Demo Rooms. However, in other environments, there may be the necessity for a different scene.
First, it would be good to check whether any of our available scenes suits your need and follow the instructions to change it.
If none of the available scenes are good, please follow [this instructions](https://shadow-experimental.readthedocs.io/en/latest/user_guide/1_arm_and_hand.html#creating-a-new-world-scene) to create one.

## Setup
### UR10
1. Unpack the robot arm and the control box.
2. Mount the robot on a sturdy surface strong enough to withstand at least 10 times
3. Place the control box on its foot.
4. Plug on the robot cable between the robot and the control box.
5. Plug in the mains plug of the control box.

To quickly start up the robot after it has been installed, perform the following steps:
1. Press the power button on the teach pendant.
2. Wait a minute while the system is starting up, displaying text on the touch screen.
3. Press ON button on the touch screen. Wait a few seconds until robot state changes to idle.
4. Press START button on the touch screen. The robot now makes a sound and moves a little while releasing the brakes.

### Hand E
1. Check the name of the hand that has to be used on the base plate of the Hand
2. Mount the hand on the UR10 by following the instructions described here.
3. Open a terminal by pressing CTRL+ALT+T, in the pop-up terminal type the following command:
   ```sh
   roscd sr_config
   ```
4. To check the configuration currently set on the laptop execute in the terminal the following command:
   ```sh
    git branch
    ```
    The current branch will show as showed in the picture below:
5. If the name that appears corresponds to the one on the label on the base of the hand DO NOT follow STEP 5 and jump straight to STEP 7 otherwise follow the next instruction.
6. To change the configuration branch to the right one run the following command:
   ```sh
    git checkout name_of_the_branch_on_the_label
    ```
7. To make sure the change has taken effect run again step 4 and check that the current branch corresponds to the one on the hand label.

### Cable connections

### Vive Headset Calibration
### Turning on and Pairing Tracker and Controller

### Set Tracker to act as Tracker

By default Steam can recognize the Tracker as a Controller depending on its position. This is quite dangerous as it may cause unwanted arm movements. Hence, these instructions MUST be executed every time a new Tracker is turned on and paired.

If for any reason it is necessary to switch tracker (e.g. power outage or damage), those steps MUST be executed. If you are not sure about it please contact someone from the Software Team.



To set the Tracker to act as a Tracker execute the following steps:

Open a Google Chrome or Firefox and in the address bar copy http://127.0.0.1:8998/dashboard/controllerbinding.html
In the page that will show up select Steam VR or Steam.
Under Current Controller, select Vive Tracker in Hand, then select Managed Trackers


## Running the system
1. Make sure all the previous steps have been executed correctly and everything is powered up.
2. Wear Cyberglove on right hand.
3. Wear Vive Tracker on right wrist.
4. Open a terminal by pressing CTRL+ALT+T
5. Run the system by executing the following command in the pop-up terminal:
   ```sh
   roslaunch sr_teleop_vive_cyberglove demo.launch
   ```
   This will start the demo.

   ```eval_rst
   .. Note:: If the HAND E has BIOTACS sensors run the following command instead of the one previously mentioned:
   ```
   ```sh
   roslaunch sr_teleop_vive_cyberglove demo.launch biotacs:=true
   ```
6. To start the hand and arm tracking, pull the trigger on the Vive Controller as showed in the picture below:
￼  ```eval_rst
   .. Note:: If the arm stopped in a strange position or for some reason you just want it to go to the HOME position press the Home button as showed in the picture below:
   ```

## FAQ

**Vive doesn't start correctly**
1. Make sure wand and controller are turned on and show green status LEDs
2. Maker sure wand and controller are visible to light houses
3. If above doesn't work, kill stop the system, close and restart Steam VR, restart the system.

**Hand and/or arm doesn't start after launching the system**
1. Unplug and plug back in to the laptop the USB Ethernet adaptors for both hand and arm. 
   * Check that the arm is reachable by pinging 192.168.1.1
   * If address is unreachable, keep plugging and unplugging the adaptors until it works.
2. Check EDC cable on hand (Goes from base → palm) is seated correctly in the palm.
   * Run eeprom tool to make sure there are 2 slaves present on the expected adaptor.
3. Restart system.

**Arm stops tracking due to fingers collision with the scene**
Pull trigger and use the cyberglove to move the finger which is in collision. The arm should be able to move again.
If the above does not work then,
Press the home button. The arm should move to the home position.
Continue as usual.

**Arm stops tracking due to self collision**
Press the home button. The arm should move to the home position.
Continue as usual.

**Arm stops tracking due to emergency stops/cut outs**
Stop the system on terminal.
Press "OK" on arm tablet.
Restart system.

**One or more of the above doesn't solve the problem.**
If in doubt, reboot.