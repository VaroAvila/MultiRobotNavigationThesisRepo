# MultiRobotNavigationThesisRepo
This repository collects my developments for my master thesis in multi robot navigation based on a fork from the ROS2 Nav2 software modules and robots from [Neobotix GmbH](https://github.com/neobotix/), mainly the parts related to simulations. It uses Ros2 Humble and has been tested on an Ubuntu 22.04.4 LTSJammy System.
<br>

## Content
<br>

In this repository you can find:
1. All the needed packages to run multi-robot simulations with the Neobotix Stack 
2. A modified plugin to command and control robots (the goals need to be inputted on the plugin's code or implement a .yaml solution to handle goals).
3. A footprint costmap layer to share robot footprints with the other robots
4. A path prediction costmap layer to share the expected path of each robot with the other robots
5. Multiple simulation scenarios, robot models and navigation configurations
<br>
## Requirements

Your system needs to have [Ros2 Humble](https://docs.ros.org/en/humble/Installation.html) and [Navigation 2](https://docs.nav2.org/getting_started/index.html) installed.

Additional modules might be needed depending on your system. 
<br>
<br>
## Instalation
<br>
Once you have the requirements installed, open your CLI and run the next set of commands:

```
cd
mkdir -p ~/ros2_humble_wss/thesis_wsp/src && cd ~/ros2_humble_wss/thesis_wsp/src
git clone https://github.com/VaroAvila/MultiRobotNavigationThesisRepo.git
```
<br>
After cloning the repository, it needs to be built by running:

```
cd ~/ros2_humble_wss/thesis_wsp
colcon build
```
<br>
Once colcon finishes building, you need to update your bashrc file to run the ros_settings.sh file everytime a CLI instance is opened - it sources the ROS2 workspace and runs some other lines to export simulation parameters. 
<br>

```
echo "source ~/ros2_humble_wss/thesis_wsp/src/ros_settings.sh" >> ~/.bashrc && tail ~/.bashrc && source ~/.bashrc
```

After this step is completed, close the console and you can start running simulations.
<br>

## Use
<br>

To run a simulation, three consoles need to be opened
<br>

1st console: Navigation. E.g.:
```
ros2 launch neo_simulation2 multi_robot_navigation_navfn.launch.py
```
2nd console: Simulation. E.g.:
```
ros2 launch neo_simulation2 multi_robot_simulation.launch.py
```
3rd console: Rviz - Visualization and GUI control. E.g.:
```
ros2 launch neo_nav2_bringup rviz_launch.py rviz_config:=install/neo_nav2_bringup/share/neo_nav2_bringup/rviz/multi_robot.rviz"
```
<br>

> [!NOTE]
> The simulation and navigation launch files can be chosen from the available list. <br>
> They can be found in the path ```~/ros2_humble_wss/thesis_wsp/src/neo_simulation2-multi-robot-sim-with-xacro/launch``` <br>
> They provide different configurations and simulations scenarios, including navigation files with the Footprint Costmap Layer and Path Prediction Costmap Layer.
> 
<br>

> [!IMPORTANT]
> Some scenarios and applications in this version are focused on the experiments and testing phase of the [MultiRobotNavigationPaperRepo](https://github.com/VaroAvila/MultiRobotNavigationPaperRepo/tree/main). <br>
> Some older scenarios and navigation files might be in files listed as "old xxxx files" or similar.






