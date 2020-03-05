# Setting up the arm

1. Unpack the robot arm and the control box.
2. Mount the arm on the table. If you have a demo table, the cable socket should point roughly towards the wire hole in the table. Place the base of the robot onto the mounting plate and add screws from the top. 
   ```eval_rst
   .. image:: ../img/ur_position_on_table.png
   ```
3. Place the control box on its foot and under the table so its not trip hazard.
   ```eval_rst
   .. image:: ../img/ur10_control_box.jpg
   ```
4. Plug in the robot cable between the robot and the control box.
   ```eval_rst
   .. image:: ../img/ur10_control_box_cables.jpg
   ```
5. Plug in the mains plug of the control box.

To quickly start up the robot after it has been installed, perform the following steps:
1. Press the power button on the teach pendant.
2. Wait a minute while the system is starting up, displaying text on the touch screen.
3. Press ON button on the touch screen. Wait a few seconds until robot state changes to idle.
4. Press START button on the touch screen. The robot now makes a sound and moves a little while releasing the brakes.

## Configuring the network

In order to use the robot with our driver you need to change the network setup of the robot via the pendant, performing the following steps:
1. To setup the IP of the robot Press Exit on Initialization screen:

   ```eval_rst
   .. image:: ../img/configure_arm_1.jpg
   ```
   
   You should see the following screen:
   ```eval_rst
   .. image:: ../img/configure_arm_2.jpg
   ```
   
2. Press the "Setup Robot" button and you should see the following screen:
   ```eval_rst
   .. image:: ../img/configure_arm_3.jpg
   ```
   
3. Then press the "Network" button. In this screen, you need to enable the network by clicking the "Static Address" radio button. Change the IP address and Subnet mask as shown below:
   * IP address: 192.168.1.1
   * Subnet mask: 255.255.255.0
   
   ```eval_rst
   .. image:: ../img/configure_arm_4.jpg
   ```
4. Press "Apply" when you finish.

## Table-Arm Calibration Procedure
Follow these steps if you have a table and a stylus provided by Shadow Robot. It should only be run once when the table is setup for the first time.

1. The arm should be mounted on the table but without hand. First, mount the calibration stylus as shown below:

   ```eval_rst
   .. image:: ../img/arm_calibration_stylus.png
     :width: 200
     :align: center
   ```

2. Attach the ar_marker base to the table without the markers on
   
3. Type CTRL+ALT+T to open a terminal and start arm with command:
    
   ```eval_rst
   .. prompt:: bash $

       roslaunch sr_robot_launch sr_ur10arm_box.launch sim:=false
   ```
   
4. Open another terminal (CTRL+ALT+T) and set the payload with the following command:
   
   ```eval_rst
   .. prompt:: bash $ 
   
       rosservice call /ra_sr_ur_robot_hw/set_payload "mass_kg: 0.0 centre_of_mass_m: x: 0.0 y: 0.0 z: 0.0"
   ```
   
5. Then, in the same terminal, change the control to `teach_mode` running the following:
   ```eval_rst
   .. prompt:: bash $
   
       rosservice call /teach_mode "teach_mode: 1 robot: 'right_arm'"
   ```
   Now you should be able to move the arm freely

6. For each marker to be calibrated:
   * Run:
     ```eval_rst
     .. prompt:: bash $
     
         roslaunch sr_workspace_calibrator calibrator.launch [calibration_frame:=FRAME_NAME]
     ```
     * For multi marker setups, `FRAME_NAME` should be unique. For a single marker setup (most cases), this can be omitted and the default name `ra_calibration_marker` will be used.
     * Follow on screen instructions, touching carefully the tip of stylus into each hole. 
       ```eval_rst
       .. image:: ../img/arm_calibration_holes.png
         :width: 300
         :align: center
       ```
       
       <br>
     
       ```eval_rst
       .. Note:: Hole 0 is nearest to arm base and numbers increase ANTI CLOCKWISE from 0.
       ```
       
   * Repeat 10 times for each hole (0,1,3).    
     
     ```eval_rst
     .. Note:: Hole 2 (shown in red) does not get probed.
     ```
  
7. The output of the process will be a yaml file named FRAME_NAME.yaml stored in sr_workspace_calibrator/config

8. Finally to finish, change the control back running the following command:
   ```eval_rst
   .. prompt:: bash $
       
       rosservice call /teach_mode "teach_mode: 0 robot: 'right_arm'"
   ```
   Now you should not be able to move the arm

If you want to use an existing calibration, a calibration tf can be broadcast by running:
```eval_rst
.. prompt:: bash $
    
    roslaunch sr_workspace_calibrator calibration_tf.launch [calibration_frame:=FRAME_NAME]
```

As before, for single marker setups, FRAME_NAME can be omitted and the default ra_calibration_marker will be used. The launch command can of course also be included in other launch files.

## UR10 supporting firmware

In the following table, you can find the firmware version of the Universal Robot software and see if it has been tested with our software:

```eval_rst
+---------------------------+-----------------------------------+-----------------------------------+--------------------+--------------------+---------------+-------------+------------+-------------------+
| UR Software Version       | User Interface                    | Robot Controller                  | Safety Processor A | Safety Processor B | Hostname      | IP address  | s/n        | Tested?           |
+---------------------------+-----------------------------------+-----------------------------------+--------------------+--------------------+---------------+-------------+------------+-------------------+
| 3.3.4.310 (Dec 06 2016)   | PolyScope 3.3.4.310 (Dec 06 2016) | URControl 3.3.4.208 (Dec 06 2016) | URSafetyA 504      | URSafetyB 256      | ur-2017304270 | 192.168.1.1 | 2017304270 | `Yes`__           |
+---------------------------+-----------------------------------+-----------------------------------+--------------------+--------------------+---------------+-------------+------------+-------------------+
| 3.4                       |                                   |                                   |                    |                    |               |             |            | Not tested        |
+---------------------------+-----------------------------------+-----------------------------------+--------------------+--------------------+---------------+-------------+------------+-------------------+
| 3.5                       |                                   |                                   |                    |                    |               |             |            | Not tested        |
+---------------------------+-----------------------------------+-----------------------------------+--------------------+--------------------+---------------+-------------+------------+-------------------+
| 3.6                       |                                   |                                   |                    |                    |               |             |            | Yes in Serfow Lab |
+---------------------------+-----------------------------------+-----------------------------------+--------------------+--------------------+---------------+-------------+------------+-------------------+
| 3.7.0.40195 (Aug 22 2018) |                                   |                                   | URSafetyA (3.5.2)  | URSafetyB (3.5.4)  | ur-2018300632 | 192.168.1.1 | 2018300632 | `Yes`__           |
+---------------------------+-----------------------------------+-----------------------------------+--------------------+--------------------+---------------+-------------+------------+-------------------+
| 3.7.2.40245 (Oct 05 2018) |                                   |                                   | URSafetyA (3.5.2)  | URSafetyB (3.5.4)  | ur-2018301419 | 192.168.1.1 | 2018301419 | Demo Room 2       |
+---------------------------+-----------------------------------+-----------------------------------+--------------------+--------------------+---------------+-------------+------------+-------------------+
| 3.8.0.61336 (Nov 22 2018) |                                   |                                   | URSafetyA (3.5.2)  | URSafetyB (3.5.4)  | ur-2018301649 | 192.168.1.1 | 2018301649 | Demo Room 2       |
+---------------------------+-----------------------------------+-----------------------------------+--------------------+--------------------+---------------+-------------+------------+-------------------+
| 3.10.0                    |                                   |                                   | URSafetyA (3.5.2)  | URSafetyB (3.5.4)  |               | 192.168.1.1 |            | Yes               |
+---------------------------+-----------------------------------+-----------------------------------+--------------------+--------------------+---------------+-------------+------------+-------------------+
| 3.11                      |                                   |                                   | URSafetyA (3.5.2)  | URSafetyB (3.5.4)  |               | 192.168.1.1 |            | Yes               |
+---------------------------+-----------------------------------+-----------------------------------+--------------------+--------------------+---------------+-------------+------------+-------------------+

__ https://shadowrobot.atlassian.net/projects/SRC?selectedItem=com.atlassian.plugins.atlassian-connect-plugin:com.kanoah.test-manager__main-project-page#!/testPlayer/SRC-R82

__ https://shadowrobot.atlassian.net/projects/SRC?selectedItem=com.atlassian.plugins.atlassian-connect-plugin:com.kanoah.test-manager__main-project-page#!/testPlayer/SRC-R83
```

Please make sure that when you test a new firmware version to update the file [known_good_firmware](https://github.com/shadow-robot/common_resources/blob/kinetic-devel/sr_firmware_checker/config/known_good_firmware.txt) with a PR adding the numbers as shown in the file.
