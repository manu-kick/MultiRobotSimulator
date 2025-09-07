# MultiRobotSimulator

![Simulator screenshot](/v5/Sim.png "Simulator Screenshot")

## DESCRIPTION
This is the MultiRobotSimulator project for the Robot Programming course from prof. Grisetti, Sapienza University of Rome. It is based on a minimal simulator created duting the course.

The goal is to use the robots (car and unicycle) to move the objects in their deignated goal area, avoiding collisions and doing it in the fastest way.
There are a few level already in the system and there is a ranking procedure that tracks the diffrent players and their matches.

## INSTRUCTIONS
Run the container 
docker run -it --rm -p 6080:80 tiryoh/ros2-desktop-vnc:humble

Open Browser on http://localhost:6080/
First thing to do is to check that the current permission on the files are read and write for your current user.
Open a new terminal, and run :
sudo chown -R  <username>: <username> <path_to_rankings_folder>
chmod u+rw <path_to_rankings_folder>/ranking.json


In the same terminal, move into the v5 directory and run:
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mrsim
source install/setup.bash

Then you run the simulator:
ros2 run mrsim simulator 

## CONTROL
Then when the simulator is running, you can run either control the robot using keyboard, following the commands below:
by default the first robot is controlled, then:
- Up/Down arrows: increase/decrease linear velocity
- Left/Right arrows: increase/decrease angular velocity
- space: stop the robot
- 1,2,3,...: switch to control robot1, robot2,...
- Tab: enable arm control, switch link selection
- e: magnet gripper on the prehensile point of an object / release the magnet
- b: return to robot control when in arm control mode
- r: reset the robot position to initial position
- esc: exit the simulator


or you can run a teleop node in another terminal. There are 3 types of teleop nodes available:
1. Teleop for robot (freeflying or car): with this you can give to the robot linear and angular velocity commands (freeflying) or steering and speed commands (car). To run it you need to publish to the topic as follows:
    ros2 topic pub -1 /robot/<robot_id>/cmd_vel geometry_msgs/Twist '{linear: {x: <float_value>}, angular: {z: <float_value>}}'
2. Teleop for arm: with this you can give to the robot arm joint position commands
    ros2 topic pub -1 /robot/<robot_id>/arm/joint_cmd sensor_msgs/JointState '{name: ["<joint_id1>","<joint_id2>","<joint_id3>"], position: [<float_value1>, <float_value2>, <float_value3>]}'
    note: ensure the robot you specify has an arm mounted
3. Teleop for gripper: with this you can give to the robot arm gripper open/close commands
    ros2 topic pub -1 /robot/<robot_id>/arm/gripper std_msgs/Bool '{data: <true/false>}'
    note: ensure the robot you specify has an arm mounted


