My research question is how does the priority of a navigation task influence a simulated robot's ability to respond? 

I am using Ubuntu 24.04, Ros2 Jazzy, and Gazebo sim. My scripts work using Emilio's robots.

My script moves the robot to simulate exploration and, during this movement, tasks are sent to the robot which have assigned urgency values. The urgency values range from 1-10, with 1 representing a non-urgent task that the robot can complete at any time and 10 representing an urgent task that the robot must attend to immediately.

Tasks are added in a queue and are carried out based on their urgency values, with higher urgency tasks being moved to the front of the queue and completed first as they are received. If a higher priority task is received while the robot is working on a lower priority task it will complete the task it is currently working on before moving to the higher priority task next. The queue is logged live to the node terminal, which when following my steps is terminal 3. 

In terminal 3, after you are done sending tasks, enter ctrl + c in order to see the time it took the robot to complete all of the tasks. The timing of the human given tasks is going to be around 1 second longer than it would actually take, as there is a baseline 1 second ACK pause for each human task.


Step 1: Launch Emilio's Robots
Terminal 1:
cd /home/Lorence-Lesniewski/Desktop/NASAproj4Wheelers
export GZ_SIM_RESOURCE_PATH=~/Desktop/NASAproj4Wheelers/SpaceSim:$GZ_SIM_RESOURCE_PATH
gz sim ~/Desktop/NASAproj4Wheelers/SpaceSim/moving_robot.sdf --verbose

Step 2: Create a bridge between Ros2 and Gazebo sim 
Terminal 2:
cd /home/Lorence-Lesniewski/Desktop/NASAproj4Wheelers/ros2_ws
source install/setup.bash
ros2 launch multi_robot_control multi_robot_bridge.launch.py

Step 3: Run the program 
Terminal 3:
source /opt/ros/jazzy/setup.bash
cd ~/Desktop/NASAproj4Wheelers/ros2_ws
source install/setup.bash
python3 src/multi_robot_control/urgency_implemented.py \
    --cmd-topic /model/robot1/cmd_vel \
    --human-topic /robot1/human_task \
    --hz 20 \
    --linear 0.4 \
    --angular 1.2 \
    --ack-pause-s 1.0
    
Step 4: Send tasks to the robot
Terminal 4:
cd ~/Desktop/NASAproj4Wheelers/ros2_ws
source install/setup.bash

Example commands to run in terminal 4:
ros2 topic pub --once /robot1/human_task std_msgs/msg/String \
"{data: '{\"task_id\":\"task_1\",\"urgency\":6,\"action\":\"forward\",\"duration\":2.5}'}"
ros2 topic pub --once /robot1/human_task std_msgs/msg/String \
"{data: '{\"task_id\":\"task_2\",\"urgency\":8,\"action\":\"backward\",\"duration\":3.0}'}"
ros2 topic pub --once /robot1/human_task std_msgs/msg/String \
"{data: '{\"task_id\":\"task_3\",\"urgency\":5,\"action\":\"pause\",\"duration\":4.0}'}"
ros2 topic pub --once /robot1/human_task std_msgs/msg/String \




