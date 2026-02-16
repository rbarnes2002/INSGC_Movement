Trey Simmons

created in:
Ubuntu 24.04
Ros2 Jazzy
Gazebo harmonic

date of last revision: 2/16/2026

Overview:
Since the 2/9/26 I have been very busy and did not get as much as I would have liked done. However, I did get the logging 
system complete and each bot will output there logs to both the console and into a logging file. Specifically the bots log
when doing the following; ignoring/requesting an interruption, accepting an interruption, starting a subtask, finishing a 
subtask (also outputs deferment time), when a subtask is interrupted for a more urgent task. This upcoming week I am going 
to change the output logging so that it better matches with the email sent 2/16.

I also got the user interruption using singular keys (courtesy of Ryan's example). 

As of Monday morning 2/16/2026 I have gotten
emilios terrain Gazebo world to work with this version of my code. 


The moving_robot.sdf is set to only spawn a two robots. If you would like to spawn all 4 robots simply delete the
"!--" from
    <!--include>
      	<uri>model://robot2</uri>
      	<name>robot2</name>
      	<pose>0 2 2 0 0 3.14159</pose>
    </include>
in the moving_robot.sdf file. 
Additionally spawn more robot controllers and change the sumOfBots variable in the server class.


How it Works:
A user and use the input terminal to send interruptions to the server. The server talks to the "bots", it sends the interruptions and 
accepts/denies bots requests. The "bots"(basically bot controllers for the bots in gazebo) requests the task for some interruption. 
The bots also directs the robots in gazebo how to do subtasks. 

Note: tasks.py is a small library I made to handle task/task creation. This mainly just helps clean up the code

How to Run:
place all relevant files in the same folder, you will then have to cd ~/ said folder

#first two steps are curtesy of Emilio.
# 1) run gazebo, new terminal
cd ~/NASAprojCurrent/SpaceSim
gz sim moving_robot.sdf --verbose

# 2) start the bridges, new terminal
cd ~/NASAprojCurrent/ros2_ws
source install/setup.bash
ros2 launch multi_robot_control multi_robot_bridge.launch.py

# 3) user intput terminal, new terminal
Note: Either run a UserInput.py (type out specific commands)
 OR run UserKeyInput.py for single key commands.
 
cd ~/Desktop/Current
source /opt/ros/jazzy/setup.bash
chmod +x UserInput.py
python3 UserInput.py

OR 

cd ~/Desktop/Current
source /opt/ros/jazzy/setup.bash
chmod +x UserKeyInput.py
python3 UserKeyInput.py

# 4) server, new terminal
cd ~/Desktop/Pub_Sub_test/src/py_pubsub/py_pubsub
source /opt/ros/jazzy/setup.bash
chmod +x server.py
python3 server.py

# 5) spawn bot controllers, new terminal for each
#bots, do this twice in order to spawn two new bots(with different names)
#THESE ARE BASICALLY BOT CONTROLLERS, NOT THE BOT THEMSELVES IN GAZEBO
cd ~/Desktop/Pub_Sub_test/src/py_pubsub/py_pubsub
export ROS_LOG_DIR=./LOGS/robot1
source /opt/ros/jazzy/setup.bash
chmod +x bot.py
python3 bot.py robot1 5 -5

export ROS_LOG_DIR=./LOGS/robot2
source /opt/ros/jazzy/setup.bash
chmod +x bot.py
python3 bot.py robot2 5 5

INTERRUPT COMMANDS, copy and paste any of these into the user input terminal to send a interruption
(Message structure: TYPE FROM TO URGENCY COMMAND PARAMS)
(moveto params: X, Y, velocity)
interruption server all 0 moveto 5 5 1
interruption server robot1 0 moveto 0 -10 1

(exploreArea: Xorigin, Yorigin, width, height, speed)
note: exploreArea creates a series of moveto subtasks that explore a grid area that starts at (Xorigin, Yorigin)
	with an area of dimensions (width, height) with a velocity of speed
interruption server all 5 exploreArea 0 0 10 10 .5
