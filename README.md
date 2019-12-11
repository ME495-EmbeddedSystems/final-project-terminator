# Final Project Terminator
### Group2: Boston Cleek, Peter Jochem, Alex Hay, Musheng He, Lin Liu


## Install Dependencies
The package depends on moveit and apriltag.
```
sudo apt install ros-melodic-moveit
```

```
sudo apt install ros-melodic-apriltag-ros
```


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
### commander
The commander node runs an action server to orchestrate the state machine.

### final_pose


### image_pipeline
The image pipeline node gets the image from baxter's hand cameras, rectifies it, and sends it to darknet. Darknet then returns a list of objects it has detected in the image. The image_pipeline node then finds the most probabale cup in the classified list and publishes this object's (x,y) pair in the image.

### move_left
This node is responsible for manipulation of the left arm. This node is responsible for picking up the gun and aiming. In order to pick up the gun the node subscribes to `/gun_pose` which contains the pose of the gun based on the april tag in the camera's frame of reference. The pose of the tag in Baxter's frame is computed using the pose of the gripper. This node subscribes to `/targetBox` which is a Point type that encodes the center of the bounding box for a cup in pixel coordinates. Aiming is accomplished by moving the gripper in Baxter's YZ plan in 1cm increments until the center of the bounding box for a cup is within a 20 pixel tolerance of the center of the camera's frame. The cameras are located at the writs of each gripper.

This node primarily takes advantage of moveit's path planning function `compute_cartesian_path()` to develop a plan between two pose goals. In `src/terminator/gripper.py` the implementation of most of moveit's functionality for manipulation can be found. The function `plan_cartesian_path()` calls `compute_cartesian_path()` and linearly interpolates a trajectory consisting of 10 waypoints between start and goal poses for a gripper. The function `compute_cartesian_path()` returns a fraction describing how much of the desired trajectory can be completed. If the planner can complete less than 80% of the trajectory the moveit planner has 3 more attempts. If the planner can not reach the threshold of 80% the moveit function `set_pose_target()` is used for manipulation. After a gripper reaches the end of its trajectory the pose is recorded using moveit's function `get_current_pose()`. If the pose is within the desired tolerances it is considered a successful move and the goal has been reached.

### move_right
This node is responsible for manipulation of the right arm. This node is responsible for pulling the trigger. The same implementations are used as described in the `move_left` node description above. In order to pull the trigger the right gripper goes to a standoff position that is perpendicular to the left gripper. From there the right gripper moves in to pull the trigger but first waits for the user to type yes in the terminal. 



## Launchfiles
image_pipeline.launch launches darknet_ros, apriltags_ros, imageproc, the image_pipeline node, and an image view node.
Darknet runs a neural network to reconize objects in images. apriltag_ros reconizes april tags in images. imageproc rectifies images. The image_pipeline node ties all the other image processing nodes together and orchestrates their interactions. The image view node launches a window to view the image classified by the neural network.     

manipulate.launch  

move.launch

setup.launch  


## Key Takeaways
Lessons learned: The whole teamed learned a lot about ROS! We also learned how to use MoveIt and its motion planning algorithms. Finally, we all realized that it takes a lot of tuning and effort to make Baxter's performace robust.


Algorithms used: Darknet ros uses a deep neural network to recognize objects
We used moveIt's computeCartesianWaypoint method to find the path between waypoints.
To aim the gun to hit the target, we simply move the gun so that the object appears in the center of the image

### Moveit
We forked the library and made a few small configuration changes. The forked repo is included in the terminator.rosinstall file

### AprilTags
We forked the apriltags_ros library and included our copy in the terminator.rosinstall file.
This library detects april tags in the enviroment. In order to use it, one needs to edit apriltag_ros/apriltag_ros/config/tags.yaml with the family, id and sizes of the tags you want the library to recognize.



## Future Work
We would like to implement tracking and shooting of objects that move through space. We would likely need to implement another method of pulling the trigger. It would be hard to move the gun and have both of the robot's hands on the trigger. To pull the trigger, we could use a rasberry pi on a wifi network and send it a signal to shoot. This would require some sort of mechatronic device.    
