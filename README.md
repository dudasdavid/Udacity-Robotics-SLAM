# Robotics Software Engineer Nanodegree Program

## Project 3/5: Where am I?

[![Where am I?](./documentation/video.png)](https://youtu.be/fnZOnAzzI0c)

[//]: # (Image References)

[image1]: ./documentation/gazebo_sim.png "Gazebo"
[image2]: ./documentation/frames.png "TF Frames"
[image3]: ./documentation/rosgraph.png "Graph"
[image4]: ./documentation/01_init.png "Localization"
[image5]: ./documentation/02_localization.png "Localization"
[image6]: ./documentation/03_navigation.png "Navigation"
[image7]: ./documentation/04_navigation.png "Navigation"
[image8]: ./documentation/05_navigation.png "Navigation"
[image9]: ./documentation/map.png "Map"

### Dependencies:
#### - PGM Map Creator
PGM Map Creator is added to the project as Git Submodule, but you can download it as an individual package:

`git clone https://github.com/udacity/pgm_map_creator.git`

#### - Teleop package
If you prefer to control your robot to help it localize itself, you can install the following teleop package:

`sudo apt install ros-melodic-teleop-twist-keyboard`

### Project build instrctions:
1. We need `libignition-math2-dev` and `protobuf-compiler` to compile the map creator:
`sudo apt-get install libignition-math2-dev protobuf-compiler`
2. Clone this repo inside the `src` folder of a catkin workspace with the PGM Map Creator submodule:
`git clone --recurse-submodules https://github.com/dudasdavid/Udacity-Robotics-RobotLocalization`
3. Build workspace: `catkin_make`
4. Source environment: `source devel/setup.bash` 
5. Start the Gazebo simulation and RViz: `roslaunch my_robot world.launch`
6. Start map server, localization and navigation: `roslaunch my_robot amcl.launch`
7. Load the RViz confuguration from: `~/catkin_ws/src/Udacity-Robotics-RobotLocalization/my_robot/rviz/localization.rviz`
8. Optionally start the Teleop package: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

### Summary of Tasks:
This project is based on the `my_robot` package of Project 2:
`https://github.com/dudasdavid/Udacity-Robotics-ChaseTheBall`.
![alt text][image1]

The static map of the Gazebo environment was saved into the map folder using the PGM Map Creator package.
![alt text][image9]
#### Saving the map:
1) Successfully build PGM Map Creator.
2) Copy the `MyWorld.world` Gazebo world file into `src/pgm_map_creator/world/` folder.
3) Add `<plugin filename="libcollision_map_creator.so" name="collision_map_creator"/>` before the `</world>` tag.
4) Run Gazebo server with: `gzserver src/pgm_map_creator/world/MyWorld.world`.
5) Save the map with PGM Map Creator: `roslaunch pgm_map_creator request_publisher.launch`.
6) Copy the saved `map.pgm` to maps folder of the `my_robot` package and create a `map.yaml` with default values (see `my_robot` package).

#### Launching the localization
With the `map.pgm` and `map.yaml` we can start the map server, AMCL localization and the move_base navigation nodes. See `my_robot/launch/amcl.launch`.

After successfully start the simulation and localization you should see the following graph after executing `rqt_graph`:
![alt text][image3]
And the following TF tree after executing `rqt_tf_tree`:
![alt text][image2]

#### Localization and navigation
1) At init the robot's pose is unknown on the map, we can see that even the yaw angle isn't correct:
![alt text][image4]
2) After manually driving a few meters the robot pose is succesfully identified by the AMCL localization:
![alt text][image5]
3) After the successful localization we can set up a navigation goal in RViz:
![alt text][image6]
4) And the robot will follow the trajectory calculated by the `move_base` navigation stack:
![alt text][image7]
5) And finally robot can successfully reach the desired navigation goal:
![alt text][image8]

### Project structure:
```bash
tree

.Udacity-Robotics-RobotLocalization             # Where am I? Project
├── README.md                                   # Project documentation
├── documentation
│   ├── 01_init.png
│   ├── 02_localization.png
│   ├── 03_navigation.png
│   ├── 04_navigation.png
│   ├── 05_navigation.png
│   ├── frames.png
│   ├── gazebo_sim.png
│   ├── map.png
│   ├── rosgraph.png
│   └── video.png
├── my_robot                                    # my_robot node
│   ├── CMakeLists.txt                          # Link libraries
│   ├── config                                  # move_base config files
│   │   ├── base_local_planner_params.yaml
│   │   ├── costmap_common_params.yaml
│   │   ├── global_costmap_params.yaml
│   │   └── local_costmap_params.yaml
│   ├── launch
│   │   ├── amcl.launch                         # Starts the map_server, amcl and move_base
│   │   ├── robot_description.launch
│   │   └── world.launch                        # Initialize robot in Gazebo environment
│   ├── maps                                    # Saved PGM map of Gazebo environment
│   │   ├── map.pgm
│   │   └── map.yaml
│   ├── meshes                                  # Custom robot meshes
│   │   ├── chassis.dae
│   │   ├── chassis.SLDPRT
│   │   ├── chassis.STEP
│   │   ├── hokuyo.dae
│   │   ├── wheel.dae
│   │   ├── wheel.SLDPRT
│   │   └── wheel.STEP
│   ├── package.xml
│   ├── rviz
│   │   ├── localization.rviz                   # RViz config file for this project
│   │   └── my_robot.rviz
│   ├── urdf                                    # Robot URDF description
│   │   ├── my_robot.gazebo
│   │   └── my_robot.xacro
│   └── worlds                                  # Simulated world in Gazebo
│       ├── empty.world
│       └── MyWorld.world
└──  pgm_map_creator                            # PGM Map Creator Git Submodule
    ├── CMakeLists.txt
    ├── launch
    │   └── request_publisher.launch
    ├── LICENSE
    ├── maps                                    # Saved PGM maps
    │   ├── example_map.pgm
    │   └── map.pgm
    ├── msgs
    │   ├── CMakeLists.txt
    │   └── collision_map_request.proto
    ├── package.xml
    ├── README.md
    ├── src
    │   ├── collision_map_creator.cc
    │   └── request_publisher.cc
    └── world                                   # Gazebo worlds
        ├── MyWorld.world
        └── udacity_mtv
```

### Remarks:
* Project was built on Ubuntu 18.04 with ROS Melodic
* PGM Map Creator had several build issues with ROS Melodic, these are fixed in the Git Submodule
* Lidar range is set to 360 degree

