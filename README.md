# final-project-terminator

# connecting and enabling 
To connect to robot and enable it
```
source src/final-project-terminator/config/connect.sh
```


export ROS_MASTER_URI=http://10.42.0.2:11311
export ROS_IP=10.42.0.1

WHEN RUNNING ON SIMULATOR&WITHOUT LAUNCHFILE:

run `roslaunch baxter_gazebo baxter_world.launch` to start the simulator

run `roslaunch baxter_moveit_config baxter_grippers.launch` to bringup robot

run `rosrun baxter_tools enable_robot.py -e` to enable the robot:

run `rosrun baxter_interface joint_trajectory_action_server.py` to start the joint trajectory controller

run `rosrun terminator aim_target` to aiming the target

WHEN RUNNING ON REAL ROBOT(make sure it's wire connected):

run `cd ~/rosfinalproject/src/terminator`to go into my package

run `source config/connect.sh `to set environment variables

to make sure environment variables have been set, run`echo $ROS_MASTER_URI`
and `echo $ROS_IP` or run `ping 10.42.0.2` make sure is getting data

run `roslaunch terminator move.launch` to bring up the moveit & start joint trajectory controller.
