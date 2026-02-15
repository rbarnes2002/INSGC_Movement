Trey Simmons

created in:
Ubuntu 24.04
Ros2 Jazzy
Gazebo harmonic

date of last revision: 2/15/2026

Overview:
This and last week I got the bots determining whether or not to request a an interruption based upon their current workload. In each of the bots 
request is the estimated deferment time and the server will now give the request to the bot that will defer it the least. One thing that I 
struggled with this week was having the robots be able to interrupt a non-urgent task with an urgent one. Prior to this week the entire system
was designed so that a subtask would have to be completed before it could change tasks. Implementing this took some time and created a few bugs 
where subtasks would either disappear or be done twice. Eventually, I was able to get everything ironed out. Since we plan on integrating 
our code soon I also refactored a large portion of my code in order to make it more readable. 

For the upcoming week:
	Add percent of exploration done as a deciding factor of which robot will address an interruption
	Better optimize the function that optimizes the subtask order, as of right now it can take a second to complete in a few cases.


The moving_robot.sdf is set to only spawn a two robots. If you would like to spawn all 4 robots simply delete the
"!--" from
    <!--include>
      	<uri>model://robot2</uri>
      	<name>robot2</name>
      	<pose>0 2 2 0 0 3.14159</pose>
    </include>
in the moving_robot.sdf file. Additionally spawn more robot controllers. 
NOTE: if you plan on changing the number of bots that you spawn you must also change a variable in the server numOfBots so that the server knows how many 
bot there are.

note: This version does have a working subtask optimization function, it is not currently used but it will be in some capacity later on. 

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
note: The 

# 2) start the bridges, new terminal
cd ~/NASAprojCurrent/ros2_ws
source install/setup.bash
ros2 launch multi_robot_control multi_robot_bridge.launch.py

# 3) user intput terminal, new terminal
cd ~/Desktop/Current
source /opt/ros/jazzy/setup.bash
chmod +x UserInput.py
python3 UserInput.py

# 4) server, new terminal
cd ~/Desktop/Pub_Sub_test/src/py_pubsub/py_pubsub
source /opt/ros/jazzy/setup.bash
chmod +x server.py
python3 server.py

# 5) spawn bot controllers, the bot controller names must match for the spawned bots above
#THESE ARE BASICALLY BOT CONTROLLERS, NOT THE BOT THEMSELVES IN GAZEBO
#the "5" params are where the bots will begin exploring
cd ~/Desktop/Pub_Sub_test/src/py_pubsub/py_pubsub
source /opt/ros/jazzy/setup.bash
chmod +x bot.py
python3 bot.py robot1 5 -5
or
python3 bot.py robot2 5 5

INTERRUPT COMMANDS, copy and paste any of these into the user input terminal to send a interruption
interruption server all 0 moveto 5 5 1
interruption server robot1 0 moveto 0 -10 1
(moveto params: X, Y, velocity)
interruption server all 0 forward 3 20
interruption server all 0 forward -3 15
interruption server robot1 0 forward 3 20

