Ryan/Riley Barnes
INSGC
How do variations in simulated terrain properties affect task completion time, and positional accuracy for a differential-drive mobile robot?

This folder contains 6 Scripts:
1) task_motion_demo.py 
2) human_task_publisher.py
3) interrupt_allocator_demo.py
4) keyboard_human_interrupt.py
5) task_metrics_logger.py
6) analyze_metrics.py

How to run, and what each file does:

Step 1) Launching the Stimulation (Courtesy of Emilio)

export GZ_SIM_RESOURCE_PATH=~/Desktop/NASAprojSubTasks/SpaceSim:$GZ_SIM_RESOURCE_PATH
gz sim ~/Desktop/NASAprojSubTasks/SpaceSim/moving_robot.sdf --verbose

This launches Emilio's simulation from desktop.
All four robots are present but we just focus on robot1 (the blue one).


Step 2) Connect Gazebo & ROS2 Nodes (Courtesy of Emilio)

cd /home/Riley/Desktop/NASAprojSubTasks/ros2_ws
source install/setup.bash
ros2 launch multi_robot_control multi_robot_bridge.launch.py

This builds and compiles the ROS2 packages of Emilio's code.
This also builds a bridge making it accessible to the next scripts.


Step 3) Make the robot "explore" in predetermined patterns

source /opt/ros/jazzy/setup.bash
cd ~/Desktop
./task_motion_demo.py --cmd-topic /model/robot1/cmd_vel --pattern both

This script publishes a twist message to /model/robot1/cmd_vel and executes motion sequences such as:
Drive forward
Turn
Repeat
Possibly square or zigzag patterns
All stimulating "exploration".


Step 4) Launch the primary decision-making controller

source /opt/ros/jazzy/setup.bash
cd ~/Desktop
./interrupt_allocator_demo.py --cmd-topic /model/robot1/cmd_vel --human-topic /robot1/human_task

This manages manages:
Baseline navigation tasks
Human interruption handling
Urgency-based preemption logic
Terrain stabilization behaviors
Geofence safety and recovery (so it does not fall off the world, set to [-8.0, 8.0] for each axis.
Event logging for experimental analysis

Step 5) Launch interruption input

source /opt/ros/jazzy/setup.bash
cd ~/Desktop
./keyboard_human_interrupt.py --topic /robot1/human_task

Simulates a human operator issuing navigation commands with varying urgency
The controls are:
Urgent interruptions:
a → Pause
b → Forward
c → Stabilize
Non-urgent interruptions:
d → Pause
e → Forward
f → Stabilize

Step 6) Record tasks & task time, etc.

source /opt/ros/jazzy/setup.bash
cd ~/Desktop
chmod +x task_metrics_logger.py
./task_metrics_logger.py

This logs the time the interruption is sent, the time it is completed, what interruption it was, etc.
It generates a CSV on the desktop to be ran through the next script.

Step 7) Interpretting the Data

source /opt/ros/jazzy/setup.bash
cd ~/Desktop
python3 analyze_metrics.py --csv ~/Desktop/robot1_metrics_[date]_[time].csv <--- change the name depending on the day

This script takes the generated CSV file and makes 3 graphs comparing how long it takes for each
kind of interruption, how it varies between interruption subtask, and how long it takes to complete
depending on exploration pattern.

NOTE: sometimes terrain stabilization (stabilize) will be labelled as "backward" as that is the task it replaced.
