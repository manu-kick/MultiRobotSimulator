# MultiRobotSimulator


Run the container 
docker run -it --rm -p 6080:80 tiryoh/ros2-desktop-vnc:humble

Open Browser on http://localhost:6080/
First thing to do is to check that the current permission on the files are read and write for your current user.
Open a new terminal, navigate into /v5/src/mrsim/rankings and run : sudo chown <username_system> ranking.json




Run the simulator using a fresh terminal in which you run from the /v5 directory:
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mrsim
source install/setup.bash

ros2 run mrsim simulator <your_path_to_config_folder>/configs/<configuration_file>.json
Then when the simulator is running, you can run either control the robot using keyboard, following the commands below:
by default the first robot (robot0) is controlled, then:
- Up/Down arrows: increase/decrease linear velocity
- Left/Right arrows: increase/decrease angular velocity
- space: stop the robot
- 1,2,3,...: switch to control robot1, robot2,...
- Tab: enable arm control, switch link selection
- e: magnet gripper on the prehensile point of an object
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


