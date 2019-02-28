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

Picture

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
5. Make sure the firmware version corresponds to the first row and second column of the Firmware version and git commit hash table.
   ```eval_rst
   If it does not or you have any doubt about this procedure, please contact someone from the Software Team
   ```
### UR10 Firmware Check

### Collision scene Check

## Setup
### UR10
### Hand E
### Cable connections
### Vive Headset Calibration
### Turning on and Pairing Tracker and Controller
### Set Tracker to act as Tracker

## Running the demo
## FAQ
Vive doesn't start correctly
Hand and/or arm doesn't start after launching the demo
Arm stops tracking due to fingers collision with the scene
Arm stops tracking due to self collision
Arm stops tracking due to emergency stops/cut outs
One or more of the above doesn't solve the problem.


