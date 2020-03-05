# Setting up a simulated system 

## Gazebo

[Gazebo](http://gazebosim.org/) is our default simultator. Follow the intructions on the next section to install and run a simulation of our robot hands using Gazebo.

### Installing the software (sim)

If you do not actually have a real hand and arm but would like to use them in simulation, then please run the following command:

ROS Kinetic (Recommended):
```eval_rst
.. prompt:: bash $

    bash <(curl -Ls bit.ly/run-aurora) docker_deploy product=hand_e
```

You can also add reinstall=true in case you want to reinstall the docker image and container. When it finishes it will show:
```bash
Operation completed
```
and it will create two icons on your desktop that you can double-click to launch the container with the system or save the log files.

### Starting a robot in simulation

First you need to start the system container by either double clicking the icon "Arm_Hand_Container" or running the following command:
```eval_rst
.. prompt:: bash $

   docker start dexterous_hand_real_hw
```
Then, inside the container, launch the arm and hand by running:
```eval_rst
.. prompt:: bash $

   roslaunch sr_robot_launch sr_right_ur10arm_hand.launch
```
