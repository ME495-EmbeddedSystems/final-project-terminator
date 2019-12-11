# Final Project Terminator
### Group2: Boston Cleek, Peter Jochem, Alex Hay, Musheng He, Lin Liu

## connecting and enabling
To connect to robot and enable it
```
source src/final-project-terminator/config/connect.sh
```


export ROS_MASTER_URI=http://10.42.0.2:11311
export ROS_IP=10.42.0.1

to make sure environment variables have been set, run`echo $ROS_MASTER_URI`
and `echo $ROS_IP` or run `ping 10.42.0.2` make sure is getting data

## Goal
Baxter picks up a nerf gun, locates a cup, pulls trigger when given a user input, and moves to a final pose.

## Videos


## Quickstart Guide
config/connect.sh (in every window except image)
image_pipeline.launch
move.launch
rosrun safe_arms

manipulate.launch 


## Action Sequence
1. Baxter goes through initial calibration and start up sequence. Arms are moved to an initial pose.
2. Baxter finds the nerf gun using an AprilTag and its left arm camera
3. Baxter moves its left arm to line up with the Baxter and closes its gripper to pick up the gun
4. Once Baxter has the gun, it uses its left arm camera to find a cup using YOLO ROS
5. Baxter keeps moving its left arm until the cup is in the center of the image produced by the camera
6. Baxter moves its right arm to put its grippers around the nerf gun trigger
7. Baxter waits for a user input to confirm the firing of the gun
8. Baxter keeps waiting until the user tells it to fire
9. Baxter pulls the trigger using its right gripper
10. Baxter moves to a final pose right after shooting


## Overall System Architecture and High Level Concepts

## Nodes
calibrate
commander
final_pose
image_pipeline
move_left
move_right

## Launchfiles


## Key Takeaways
Lessons learned, algorithms used, etc
### Moveit


### AprilTags




## Future Work
