# Installing the software

We have created a one-liner that is able to install Docker, download the image and create a new container for you. It will also create two desktop icons, one to start the container and launch the hand and another one to save the log files locally. To use it, you first need to have a PC with Ubuntu installed on it (preferable version 16.04) then follow these steps:

## Check your hand interface ID

Before setting up the docker container, the EtherCAT interface ID for the hand needs to be discovered. In order to do so, after plugging the hand’s ethernet cable into your machine and powering it up, please run

```eval_rst
.. prompt:: bash $

   sudo dmesg
```
command in the console. At the bottom, there will be information similar to the one below:

```bash
[490.757853] IPv6: ADDRCONF(NETDEV_CHANGE): enp0s25: link becomes ready
```
In the above example, ‘enp0s25’ is the interface ID that is needed.

## Get ROS Upload login credentials

If you want to upload technical logged data (ROS logs, backtraces, crash dumps etc.) to our server and notify the Shadow's software team to investigate your bug then you need to enable logs uploading in the one-liner. In order to use this option you need to obtain a unique upload key by emailing sysadmin@shadowrobot.com. When you receive the key you can use it when running the one-liner installation tool. To enable the logs uploading you need to add the command line option ```-ck true``` to the one-liner.
After executing the one-liner, it will prompt you to enter your upload key and press enter to continue. Pleaser copy and paste your key from the email you received by Shadow Robot.

## Check your hand configuration branch

You should have the name of your [sr_config](https://github.com/shadow-robot/sr-config) hand branch which contains the specific configuration of your hand (calibration, controller tuning etc…).
Usually it is something like this: ``shadowrobot_XXXXXX``. Where XXXXXX are the 6 digits contained in the serial number of the hand labelled underneath the robot base.

If you are unsure please contact us.

## Run the one-liner

The one-liner will install Docker, pull the image from Docker Hub, and create and run a container with the parameters specified. In order to use it, use the following command:

```eval_rst
.. Note:: Please remember to replace [EtherCAT interface ID] with your Interface ID and [sr_config_branch] with your unique sr_config branch
```

ROS Kinetic (Recommended):

```eval_rst
.. prompt:: bash $

    bash <(curl -Ls http://bit.ly/launch-sh) -i shadowrobot/dexterous-hand:kinetic-release -n arm-and-hand -sn Arm_Hand_Launcher -e [EtherCAT interface ID] -b [sr_config_branch] -l false
```

Examples:
For Interface ID ```ens0s25``` and sr_config_branch ```shadow_12345```

```eval_rst
.. prompt:: bash $

    bash <(curl -Ls http://bit.ly/launch-sh) -i shadowrobot/dexterous-hand:kinetic-release -n arm-and-hand -sn Arm_Hand_Launcher -e ens0s25 -b shadow_12345 -l false
```  

Same as above but with ROS logs upload enabled

```eval_rst
.. prompt:: bash $

    bash <(curl -Ls http://bit.ly/launch-sh) -i shadowrobot/dexterous-hand:kinetic-release -n arm-and-hand -sn Hand_Launcher -e ens0s25 -b shadow_12345  -l false -ck true 
```  

If you have an Nvidia graphics card, you can add -nv to set the nvidia-docker version. Use ``-nv 1`` or ``-nv 2`` for version 1.0 or 2.0 respectively. Must be used with ``-g true``.

You can also add -r true in case you want to reinstall the docker image and container. 

When the one-liner finishes it will show:

```bash
Operation completed
```

and it will create two desktop icons on your desktop that you can double-click to launch the system or save the log files from the active containers to your desktop.

# Starting the driver

Launch the driver at a terminal (in the container), typing:

```eval_rst
.. prompt:: bash $

    roslaunch sr_ethercat_hand_config sr_system.launch
```

## Lights in the hand
When the ROS driver is running you should see the following lights on the Palm:

```eval_rst
========================   =============       ================    =================================
Light                      Colour              Activity            Meaning
========================   =============       ================    =================================
Run                        Green               On                  Hand is in Operational state
CAN1/2 Transmit            Blue                V.fast flicker      Demand values are being sent to the motors
CAN1/2 Receive             Blue                V.fast flicker      Motors are sending sensor data
Joint sensor chip select   Yellow              On                  Sensors being sampled
========================   =============       ================    =================================
```

After killing the driver, the lights will be in a new state:
```eval_rst
========================   =============       ================    =================================
Light                      Colour              Activity            Meaning
========================   =============       ================    =================================
Run                        Green               Blinking            Hand is in Pre-Operational state
CAN1/2 Transmit            Blue                Off                 No messages transmitted on CAN 1/2
CAN1/2 Receive             Blue                Off                 No messages received on CAN 1/2
Joint sensor chip select   Yellow              Off                 Sensors not being sampled
========================   =============       ================    =================================
```

# Saving log files and uploading data to our server
When running the one-liner, you will also notice a second icon named `Save logs` that is used to retrieve and copy all the available logs files from the active containers locally on your Desktop. This icon will create a folder that matches the active container's name and the next level will include the date and timestamp it was executed. When it starts, it will prompt you if you want to continue, as by pressing yes it will close all active containers. After pressing "yes", you will have to enter a description of the logging event and will start coping the bag files, logs and configuration files from the container and then exit. Otherwise, the window will close and no further action will happen. If you provided an upload key with the one-liner installation then the script will also upload your LOGS in compressed format to our server and notify the Shadow's software team about the upload. This will allow the team to fully investigate your issue and provide support where needed.
