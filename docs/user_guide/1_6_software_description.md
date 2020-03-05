# Software description

Check the `Software description` section in the [Dexterous Hand Documentation](https://dexterous-hand.readthedocs.io/en/latest/user_guide/2_software_description.html#), as it contains important information of all the software available for the hand.

## Repositories

Our code is split into different repositories:

* [sr_common](https://github.com/shadow-robot/sr_common): This repository contains the bare minimum for communicating with the Shadow Hand from a remote computer (urdf models and messages).
* [sr_core](https://github.com/shadow-robot/sr_core): These are the core packages for the Shadow Robot hardware and simulation.
* [sr_interface](https://github.com/shadow-robot/sr_interface): This repository contains the high level interface and its dependencies for interacting simply with our robots.
* [sr_tools](https://github.com/shadow-robot/sr_tools): This repository contains more advanced tools that might be needed in specific use cases.
* [sr_visualization](https://github.com/shadow-robot/sr_visualization): This repository contains the various rqt_gui plugins we developed.
* [sr_config](https://github.com/shadow-robot/sr_config): This repository contains the customer specific configuration for the Shadow Robot Hand.

## Robot commander

The robot commander provides a high level interface to easily control the different robots supported by Shadow Robot. It encapsulates functionality provided by different ROS packages, especially the moveit_commander, providing access via a simplified interface.

There are three clases available:
* [SrRobotCommander](https://github.com/shadow-robot/sr_interface/blob/kinetic-devel/sr_robot_commander/src/sr_robot_commander/sr_robot_commander.py): base class. Documentation can be found in the following [link](https://dexterous-hand.readthedocs.io/en/latest/user_guide/2_software_description.html#srrobotcommander).
* [SrHandCommander](https://github.com/shadow-robot/sr_interface/blob/kinetic-devel/sr_robot_commander/src/sr_robot_commander/sr_hand_commander.py): hand management class. Documentation can be found in the following [link](https://dexterous-hand.readthedocs.io/en/latest/user_guide/2_software_description.html#srhandcommander).
* [SrArmCommander](https://github.com/shadow-robot/sr_interface/blob/kinetic-devel/sr_robot_commander/src/sr_robot_commander/sr_arm_commander.py): hand management class

### SrArmCommander

The SrArmCommander inherits all methods from the [robot commander](https://dexterous-hand.readthedocs.io/en/latest/user_guide/2_software_description.html#srrobotcommander) and provides commands specific to the arm. It allows movement to a certain position in cartesian space, to a configuration in joint space
or move using a trajectory.

#### Setup
```eval_rst
Import the arm commander along with basic rospy libraries and the arm finder:

.. code:: python

    import rospy
    from sr_robot_commander.sr_arm_commander import SrArmCommander
    from sr_utilities.arm_finder import ArmFinder

The constructors for ``SrArmCommander`` take a name parameter that should match the group name of the robot to be used and has the option to add ground to the scene.

.. code:: python

   arm_commander = SrArmCommander(name="right_arm", set_ground=True)
   
Use the ArmFinder to get the parameters (such as prefix) and joint names of the arm currently running on the system:

.. code:: python

   arm_finder = ArmFinder()
   
   # To get the prefix or mapping of the arm joints. Mapping is the same as prefix but without underscore.
   arm_finder.get_arm_parameters().joint_prefix.values()
   arm_finder.get_arm_parameters().mapping.values()
   
   # To get the arm joints
   arm_finder.get_arm_joints()
```

#### Getting basic information
```eval_rst
To return the reference frame for planning in cartesian space:

.. code:: python

   reference_frame = arm_commander.get_pose_reference_frame()
```

#### Plan/move to a position target
```eval_rst
Using the method ``move_to_position_target``, the end effector of the arm can be moved to a certain point
in space represented by (x, y, z) coordinates. The orientation of the end effector can take any value.

Parameters:

-  *xyz* desired position of end-effector
-  *end\_effector\_link* name of the end effector link (default value is
   empty string)
-  *wait*  indicates if the method should wait for the movement to end or not
   (default value is True)
```
##### Example
```eval_rst

.. code:: python

   rospy.init_node("robot_commander_examples", anonymous=True)
   arm_commander = SrArmCommander(name="right_arm", set_ground=True)

   new_position = [0.25527, 0.36682, 0.5426]
    
   # To only plan
   arm_commander.plan_to_position_target(new_position)
    
   # To plan and move
   arm_commander.move_to_position_target(new_position)
```

#### Plan/move to a pose target
```eval_rst
Using the method ``move_to_pose_target`` allows the end effector of the arm to be moved to a certain pose
(position and orientation) in the space represented by (x, y, z, rot\_x,
rot\_y, rot\_z).

Parameters:

-  *pose* desired pose of end-effector: a Pose message, a PoseStamped
   message or a list of 6 floats: [x, y, z, rot\_x, rot\_y, rot\_z] or a
   list of 7 floats [x, y, z, qx, qy, qz, qw]
-  *end\_effector\_link* name of the end effector link (default value is
   empty string)
-  *wait* indicates if the method should wait for the movement to end or not
   (default value is True)
```
##### Example
```eval_rst

.. code:: python

   rospy.init_node("robot_commander_examples", anonymous=True)
   arm_commander = SrArmCommander(name="right_arm", set_ground=True)

   new_pose = [0.5, 0.3, 1.2, 0, 1.57, 0]
   
   # To only plan
   arm_commander.plan_to_pose_target(new_pose)
   
   # To plan and move
   arm_commander.move_to_pose_target(new_pose)

```
## Creating a new world/scene

In this section, instructions on how to create, modify and save new `.world` and `.scene` file are provided. All the necessary console commands are described in depth, however, it is recommended that the user uses the graphical user interface introduced at the end of this section.

### Using console commands

#### Running template world file

In order to start creating a new world file, first you need to run a launch file with a template world file, i.e.:

```eval_rst
.. prompt:: bash $

   roslaunch sr_world_generator create_world_template.launch
```
This will open Gazebo and Rviz with a robot in place:

```eval_rst
.. image:: ../img/empty_world.png
  :width: 400
  :align: center
```
In most cases, when one of Shadow's robot tables is used, the above command will suffice. However, the launch file can be run with multiple arguments. Arguments available for the launch file:
* **start_home** - if set to `true`, robot will start in a predefined home pose. Default value: `true`
* **scene** - if set to `true`, a scene from world file defined by the world argument will be generated. Default value: `false`
* **initial_z** - value defining positioning of the robot base in the world frame. Default value: `0.7751`
* **world** - path to a world file that will be spawned in gazebo after running the launch file. No default value, needs to be explicitly specified if `scene` argument is set to `true`.

As an example, a launch file starting with robot NOT in home position with a base at 0.5m height would be called as follows:

```eval_rst
.. prompt:: bash $

   roslaunch sr_world_generator create_world_template.launch start_home:=false initial_z:=0.5
```

#### Adding objects to the world

In order to add an existing object to the world, navigate to the left hand side bar in Gazebo and click on the **Insert** tab:

```eval_rst
.. image:: ../img/insert_object.png
  :width: 400
  :align: center
```

A list of objects will appear. Please do not use other objects than the ones kept in sr_description_common (i.e. the ones in second drop down on the list):

```eval_rst
.. image:: ../img/object_list.png
  :width: 400
  :align: center
```

In order to add an object, click on its name and move the cursor back to the scene. A shadow of the object will appear that you can move around. Single left click will put the object in a specified location.

```eval_rst
.. image:: ../img/table_added.png
  :width: 400
  :align: center
```

In order to move the object around, click on the object, then click the following icon found at the top of the panel,

```eval_rst
.. image:: ../img/move_object.png
  :width: 400
  :align: center
```

then click back on the object. You can move it around now. It is usually easier to use the appearing axis frame instead of trying to drag the object itself.

The same process can be done for rotation, after clicking this icon:

```eval_rst
.. image:: ../img/rotate_object.png
  :width: 400
  :align: center
```

It is also possible to set the specific pose of the object in the pose field. You can do that by clicking the object, navigating to the `pose` drop-down on the left-hand side bar and setting desired pose.

```eval_rst
.. image:: ../img/pose_change.png
  :width: 400
  :align: center
```

A video depicting the process described above can be found [here](https://drive.google.com/file/d/1bm6PckbXbUY9ELF_6f4LWAIdXbkIZnQ1/view?usp=sharing).

#### Creating new objects

It is possible to create new object types from both meshes and primitives. First, an object needs to be placed in the scene. You can either drag a mesh that you want to modify as described above or use one of the available primitives that you can see at the top of the panel:

```eval_rst
.. image:: ../img/primitives.png
  :width: 400
  :align: center
```

In this example, we will be using a primitive to create a wall. After inserting the primitive in a scene, it's dimensions can be changed. In order to do that, right click on the model you just inserted and select **Edit model** option:

```eval_rst
.. image:: ../img/edit_model.png
  :width: 400
  :align: center
```

Further, right click the object again and select **Open Link Inspector**.

```eval_rst
.. image:: ../img/link_inspector.png
  :width: 400
  :align: center
```

Inside the **Link Inspector**, go to the **Visual** tab, scroll down to **Geometry** section and select desired dimensions. Further, go to **Collision** tab and do the same. Finally, click OK to confirm the changes. In the below example, a 1x1x1m square was reduced to a thin wall:

```eval_rst
.. image:: ../img/modified_model.png
  :width: 400
  :align: center
```

Next step is to save the model. Go to **File → Save as**. A pop-up window will show:

```eval_rst
.. image:: ../img/save_model.png
  :width: 400
  :align: center
```

For model name, DO NOT use numbers. Other than that, any name would suffice, provided it does not already exist in the models folder. In order to save the model in a proper location, use the **Browse** button and navigate to `models` folder in `sr_description_common` package (should be either `/home/user/projects/shadow_robot/base_deps/src/common_resources/sr_description_common/models` or `/home/user/projects/shadow_robot/base/src/common_resources/sr_description_common/models` directory). When you are ready, use the **Save** button to finish. Your model has now been saved. Go to **File → Exit Model Editor** to close the model editor. Now you can move and rotate the object as discussed in the section above.

As mentioned before, the same process (dimension change and saving) can be used with mesh files.

A video depicting the process described above can be found [here](https://drive.google.com/file/d/1yoLMEdtsf-U4bimTCqofrLLD3mPABT-9/view?usp=sharing).

#### Generating proper world file

When all the models are inserted in the scene and placed in desired position, the world file can be saved. Go to **File → Save World As** and select a name and a path of a world file saved with gazebo. Make sure to remember the path to the file. We recommend using the path `/home/user`. Although the file has now been saved, it has to be modified before being used by our launch files. In order to modify it, first kill the currently running Gazebo launch file and run:

```eval_rst
.. prompt:: bash $

   roslaunch sr_world_generator save_world_file.launch gazebo_generated_world_file_path:=path_to_file output_world_file_name:=file_name
```

where:
* **path_to_file** - path to file that was previously saved through gazebo,
* **file_name** -  desired name of the file (without extension). If not set, defaults to `new_world`.

When a message `World saved!` will appear in the console, kill the launch file. Your world has now been saved in `sr_description_common` package (`worlds` folder) and is ready to be used.

#### Creating a scene file

In order to generate a scene file for collision scene used in non-simulated scenarios, first run the initial launch file with the just created world file passed to the `world` argument:

```eval_rst
.. prompt:: bash $

   roslaunch sr_world_generator create_world_template.launch scene:=true world:=path_to_world
```

where **path_to_world** is the full path to the world file that just has been generated. When Rviz starts, on the left hand side, navigate to the **Scene Objects** tab

```eval_rst
.. image:: ../img/create_scene.png
  :width: 400
  :align: center
```

and click **Export As Text**. A pop-up window will appear asking for a name and path for the file. It is recommended that the file is saved in the sr_description_common package, scenes folder and it's name is the same as the corresponding world file.

A video depicting the process described above can be found [here](https://drive.google.com/file/d/1Uv1MeC2xc1nZ8Ati1cKaegHN8LJzsyhM/view?usp=sharing).

### Using the graphical user interface

A GUI has been implemented to help with the above operations.

```eval_rst
.. image:: ../img/world_generator_gui.png
  :width: 400
  :align: center
```

In order to start it, make sure no Gazebo sessions are up and run:

```eval_rst
.. prompt:: bash $

   roslaunch sr_world_generator world_generator_gui.launch
```

In order to start a new Gazebo session set following parameters to your preference:
* **start home** - choose `yes` if you want the robot to be starting in it's home pose
* **empty world** - choose `yes` if you want to start with an empty world. Choose `no` if you want to start the session with a specific world file loaded. You can type the path to the world file in the edit box or navigate to the file using the `browse` button
* **initial z** - set `z` position of the robot base. Default value corresponds to tables used at Shadow

After setting the above parameters to your preference, click `Open`. A new session of Gazebo will start. After modifying the world to your liking, as in the instructions in the previous sections, go to `File → Save World As` and select a name and a path of a world file. Make sure to remember the path to the file. We recommended the path `/home/user`. Although the file has now been saved, it has to be modified before being used by our launch files. In order to do that, first kill current Gazebo session using the `Close` button in the `Open Gazebo` section of the GUI. Then use the `Transform world file` area to navigate to your newly created Gazebo world file and click `Transform`. A pop-up window will appear asking for the properly formatted world file name. After clicking `Save` your file will be created and will be ready to be used.

You can use the `Open Gazebo` section again to check your newly created world file and export it to the `scene` file as described in the sections above.

## Sensor information
### Force/Torque feedback (only applicable for UR10e)
The UR10e comes equipped with a force/torque sensor on the end effector with the following specifications:

```eval_rst
+----------------------------+---------+
| F/T Sensor - Force, x-y-z  |         |
+----------------------------+---------+
| Range                      | 100 N   |
+----------------------------+---------+
| Resolution                 | 2.0 N   |
+----------------------------+---------+
| Accuracy                   | 5.5 N   |
+----------------------------+---------+

+----------------------------+---------+
| F/T Sensor - Torque, x-y-z |         |
+----------------------------+---------+
| Range                      | 10 Nm   |
+----------------------------+---------+
| Resolution                 | 0.02 Nm |
+----------------------------+---------+
| Accuracy                   | 0.60 Nm |
+----------------------------+---------+
```

To read the data from the sensor use the topic:
```eval_rst
.. prompt:: bash $

   rostopic echo /wrench
```

It is of type WrenchStamped as defined [here](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/WrenchStamped.html).
