Emilio Rodriguez

Instructions on how to run collision avoidance script with Lorence's urgency node:

robot_pose_aggregator.py — A ROS 2 node that reads all robot and selected obstacle poses from Gazebo and publishes them on a single topic.

Run gz sim and multi_robot_bridge as usual

In new terminal, start robot_pose_aggregator.py. Must be run after gz sim and bridge

Logger is optional

Then run Lorence's urgency node for each robot. Example...

For robot1:
python3 Lorence/urgency_implemented.py   --cmd-topic /model/robot1/cmd_vel   --human-topic /robot1/human_task   --events-topic /task_events   --robot-name robot1   --poses-topic /robot_poses   --avoidance-radius 1.2   --obstacle-radius 4.0   --hz 20 --linear 0.4 --angular 1.2 --ack-pause-s 1.0

For robot2:
python3 Lorence/urgency_implemented.py   --cmd-topic /model/robot2/cmd_vel   --human-topic /robot2/human_task   --events-topic /task_events   --robot-name robot2   --poses-topic /robot_poses   --avoidance-radius 1.2   --obstacle-radius 4.0   --hz 20 --linear 0.4 --angular 1.2 --ack-pause-s 1.0

For robot3:
python3 Lorence/urgency_implemented.py   --cmd-topic /model/robot3/cmd_vel   --human-topic /robot3/human_task   --events-topic /task_events   --robot-name robot3   --poses-topic /robot_poses   --avoidance-radius 1.2   --obstacle-radius 4.0   --hz 20 --linear 0.4 --angular 1.2 --ack-pause-s 1.0

For robot4:
python3 Lorence/urgency_implemented.py   --cmd-topic /model/robot4/cmd_vel   --human-topic /robot4/human_task   --events-topic /task_events   --robot-name robot4   --poses-topic /robot_poses   --avoidance-radius 1.2   --obstacle-radius 4.0   --hz 20 --linear 0.4 --angular 1.2 --ack-pause-s 1.0

