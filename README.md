# UR5 ROS1 MoveIt Pick And Place Project
## Description: 
###This project used ROS1 Noetic on a machine running Ubuntu 20.04.
You will have to install ROS1 Noetic on Ubuntu 20.04 if you don't already for this project to work.###
Everything else is contained within this repo.

### How to set up and run:
1. create a new workspace e.g `mkdir ur5_project_ws`
2. `cd ur5_project_ws`
3. clone the src folder of this repo into the folder
4. Build with: `catkin_make`
5. Open two terminator terminals
6. source workspace: `source devel/setup.bash`
7. launch with `roslaunch ur5_moveit demo_gazebo.launch`
8. In anothor terminal go to scripts folder in ur5_moveit package
9. Run script main `python3 main.py`

![Screenshot from 2024-09-23 16-24-52](https://github.com/user-attachments/assets/55b7154a-a06e-4c98-aff2-a83aba658158)


   
