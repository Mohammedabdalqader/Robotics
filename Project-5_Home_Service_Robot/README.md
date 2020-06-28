# Final Project - Home Service Robot


## Project Description  
Directory Structure  
```
.Home-Sevice-Robot                                        # Home Service Robot Project
├── catkin_ws                                             # Catkin workspace
│   ├── src
│   │   ├── my_robot                                      
│   │   │   ├──
│   │   │   ├──
│   │   ├── add_markers                                   # add_markers package        
│   │   │   ├── launch
│   │   │   │   ├── add_markers.launch                    # launch file for home service robot
│   │   │   ├── src
│   │   │   │   ├── add_markers.cpp                       # source code for add_markers node
│   │   ├── pick_objects                                  # pick_objects package 
│   │   │   ├── launch
│   │   │   │   ├── pick_objects .launch                
│   │   │   ├── src
│   │   │   │   ├── pick_objects.cpp                      # source code for pick_objects node
│   │   ├── rvizConfig                                    # rvizConfig package        
│   │   │   ├── home_service_rvizConfig.rviz              # rvizConfig file for home service robot demo  
│   │   ├── scripts                                       # shell scripts files
│   │   │   ├── add_marker.sh                             # shell script to model virtual objects  
│   │   │   ├── home_service.sh                           # shell script to launch home service robot demo  
│   │   │   ├── pick_objects.sh                           # shell script to send multiple goals  
│   │   │   ├── test_navigation.sh                        # shell script to test localization and navigation
│   │   │   ├── test_slam.sh                              # shell script to test SLAM
│   │   ├── slam_gmapping                                 # gmapping_demo.launch file
│   │   ├── turtlebot                                     # keyboard_teleop.launch file
│   │   ├── turtlebot_interactions                        # view_navigation.launch file
│   │   ├── turtlebot_simulator                           # turtlebot_world.launch file package        
│   │   ├── CMakeLists.txt                                # compiler instructions

```

### Ofiicial ROS Packages

  * gmapping:
  
    With the gmapping_demo.launch file, you can easily perform SLAM and build a map of the environment with a robot equipped with laser range finder sensors or RGB-D cameras.
  * turtlebot_teleop:
  
    With the keyboard_teleop.launch file, you can manually control a robot using keyboard commands.
  * turtlebot_rviz_launchers:
  
    With the view_navigation.launch file, you can load a preconfigured rviz workspace. You’ll save a lot of time by launching this file, because it will automatically load the robot model, trajectories, and map for you.
  * turtlebot_gazebo:
  
    With the turtlebot_world.launch you can deploy a turtlebot in a gazebo environment by linking the world file to it.

### Custom ROS Packages
   * pick_objects:
   
     This node will send multiple goals for the robot to reach.  
     The robot travels to the desired pickup zone, displays a message that it reached its destination, waits 5 seconds, travels to the desired drop off zone, and displays a message that it reached the drop off zone."  
   
   * add_markers:
   
     This node will publish a marker to rviz.  
     The marker should initially be published at the pickup zone. After 5 seconds it should be hidden. Then after another 5 seconds it should appear at the drop off zone.

   
### Mapping  
You will create a `test_slam.sh` script file and launch it to manually test SLAM. Then a functional map of the environment should be created which would be used for localization and navigation tasks.

```bash
#!/bin/sh
xterm -e "source devel/setup.bash;roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "source devel/setup.bash;roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
xterm  -e  "source devel/setup.bash;roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "source devel/setup.bash;roslaunch turtlebot_teleop keyboard_teleop.launch"


```

### Localization and Navigation  
You will create a `test_navigation.sh` script file to launch it for manual navigation test.  
Your robot should be able to navigate in the environment after a 2D Nav Goal command is issued.  

```bash
#!/bin/sh
xterm -e "source devel/setup.bash;roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "source devel/setup.bash;roslaunch turtlebot_gazebo amcl_demo.launch" & 
sleep 5
xterm  -e  " source devel/setup.bash;roslaunch turtlebot_rviz_launchers view_navigation.launch" &

```
Then you will create a `pick_objects.sh` file that will send multiple goals for the robot to reach.  
The robot travels to the desired pickup zone, displays a message that it reached its destination, waits 5 seconds, travels to the desired drop off zone, and displays a message that it reached the drop off zone."  

```bash
#!/bin/sh
xterm -e "source devel/setup.bash;roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "source devel/setup.bash;roslaunch turtlebot_gazebo amcl_demo.launch"  &
sleep 5
xterm -e " source devel/setup.bash;roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm -e " source devel/setup.bash;roslaunch pick_objects pick_objects.launch "
```

### Home Service Functions  
Here you will create a `add_marker.sh` file that will publish a marker to rviz.  
The marker should initially be published at the pickup zone. After 5 seconds it should be hidden. Then after another 5 seconds it should appear at the drop off zone.

```bash
#!/bin/sh
xterm -e "source devel/setup.bash;roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "source devel/setup.bash;roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "source devel/setup.bash;roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "source devel/setup.bash;roslaunch add_markers add_markers.launch" &
sleep 5
xterm -e "source devel/setup.bash;roslaunch pick_objects pick_objects.launch"
```

## Run the project
I assume that you have ROS installed on your machine and all necessary dependencies.

Open your Terminal
```bash

mkdir -p catkin_ws/src
cd src/
catkin_init_workspace

git clone https://github.com/Mohammedabdalqader/Robotics.git
mv Robotics/Project-5_Home_Service_Robot/* ./ && rm -rf Robotics
cd ..
catkin_make
source devel/setup.bash
```

* Launch the home service robot
```
./src/scripts/home_service.sh
```
Done! 
