#!/bin/bash
rosrun baxter_examples send_urdf_fragment.py -f /home/bostoncleek/rethink/src/baxter_common/baxter_description/urdf/left_end_effector.urdf.xacro -l left_hand -j left_gripper_base &
rosrun baxter_examples send_urdf_fragment.py -f /home/bostoncleek/rethink/src/baxter_common/baxter_description/urdf/right_end_effector.urdf.xacro -l right_hand -j right_gripper_base &
