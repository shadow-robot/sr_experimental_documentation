# Setting up a simulated system 

## Gazebo

[Gazebo](http://gazebosim.org/) is our default simultator. So follow the intructions on the next section to install and run a simulation of our robot hands using Gazebo.

### Installing the software (sim)

If you do not actually have a real hand and arm but would like to use them in simulation, then please run the following command:

ROS Kinetic (Recommended):
```eval_rst
.. prompt:: bash $

    bash <(curl -Ls http://bit.ly/launch-sh) -i shadowrobot/dexterous-hand:kinetic-release -n arm-and-hand -sn Arm_Hand_Container -b kinetic_devel -e eth0 -l false
```

You can also add -r true in case you want to reinstall the docker image and container. When it finishes it will show:
```bash
Operation completed
```
and it will create two icons on your desktop that you can double-click to launch the container with the system or save the log files.

### Starting a robot in simulation

First you need to start the system container by either doble clicking the icon "Arm_Hand_Container" or running the following command:
```eval_rst
.. prompt:: bash $

   docker start arm-and-hand
```
Then, inside the container, launch the arm and hand by running:
```eval_rst
.. prompt:: bash $

   roslaunch sr_robot_launch sr_right_ur10arm_hand.launch
```
