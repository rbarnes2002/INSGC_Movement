import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
#import tf_transformations.euler_from_quaternion
import copy
import time
import sys
import threading
import Tasks
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, Duration
import math


class botNode(Node):

    def __init__(self, botString, explX, explY):
    
        self.botName = botString
    
    	#listener/subscriber section, listens to serverPublish
        super().__init__(botString,  namespace= f'/{botString}')
        self.botListenerCallBackGroup = MutuallyExclusiveCallbackGroup()
        self.subscription = self.create_subscription(String, '/ServerPublish', 
    	        self.botListener, 
                100, 
                callback_group = self.botListenerCallBackGroup)
    	        
        self.subscription  # prevent unused variable warning
        
        
        #quality of service profile
        qos_Profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                                history=HistoryPolicy.KEEP_LAST,
                                                depth = 10,
                                                lifespan = Duration(seconds = 10))
        
        self.poseCallBackGroup = MutuallyExclusiveCallbackGroup()
        #listens to gazebos odom for bot position
        self.poseSubscription = self.create_subscription(PoseStamped, 
                                                                               f"/model/{botString}/pose",
                                                                               self.UpdateBotLocation,
                                                                               qos_Profile,
                                                                               callback_group = self.poseCallBackGroup)
        self.poseSubscription
        
        
    	#publisher section, publishes to the server
        self.publisher_ = self.create_publisher(String, '/BotPublish', 100) 
        
        #publishes to the logging system
        self.logPublisher = self.create_publisher(String, "/task_events", 10)
        
        #publishes twist commands for bot movement
        self.movementPublisher = self.create_publisher(Twist, f"/model/{self.botName}/cmd_vel", 10)
    	
    	#list of subtasks
        self.listOfTasks = []
    	
    	#data for exploration params
        self.exploreX = explX
        self.exploreY = explY
        self.exploreWidth = 3
        self.exploreHeight = 3
   
    	#initialize exploration
        self.listOfTasks += Tasks.CreateExploreTasks(self.botName, 0, 1, "Explore_"+ self.botName, 
                                                  self.exploreX, self.exploreY, self.exploreWidth, self.exploreHeight)
        
        #task tracker for the main explore task
        self.listOfTaskTrackers = [Tasks.taskTracker(taskID = "Explore_" + self.botName, numOfSubtasks = len(self.listOfTasks))]
        
        #odometry of the bot
        self.BotLocation = botLocation()
        
        #get pose
        x, y, z, yaw = Tasks.getPoseHelper(botString)
        #optimize explore
        self.listOfTasks = Tasks.orderSubtasks(self.listOfTasks, x, y)
        
        self.currSubtask = None #self.listOfTasks.pop(0)#current subtask being done
        #the coords at the end of the current subtask
        self.currFinalX = self.listOfTasks[0].finishX
        self.currFinalY = self.listOfTasks[0].finishY
        self.currUrgency = self.listOfTasks[0].urgency#current highest priority level in the subtask list
        
        #used to avoid collisions, this is the action needed to be done in order to avoid a collision
        self.collisionType = "nothing" #nothing indicates no action is needed to avoid a collision
    	
    	# prevent logging the same interruption multiple times
        # self.interruptedCurrentTask = False

    	#user interruption handling, used to check if a bot should request some interruption
        self.interruptLoad = 0.0
        self.maxInterruptLoad = 100.0 #maximum number of user interrupts the robot will have (including current ones)
        self.interruptLoadDecay = 1.0 #each time a user task is done, interruptLoad is decreased by this much
    	
    	#this section is designed to run a check for tasks
        self.SubtaskListLock = threading.Lock()
        self.checkSubTaskTimer = self.create_timer(.5, self.checkSubTask)
        
        #for data collection/aggregate data to send to the logger
        self.idleTime: float = 0.0
        self.idleTimeStart: float = None
        
        self.numOfExploreTasksComplete: int = 0
        self.numOfExploreTasks: int = len(self.listOfTasks)
        
        self.SendAggregateLog()#this lets the logger know that this bot exists
        
    #listens to and responds to server
    def botListener(self, msg):
        TYPE, FROM, TO, URGENCY, PRIORITY, ID, TASK, PARAMS = Tasks.ParseMsg(msg.data)
        
        #DEBUG
        self.get_logger().info(f"{TYPE} {FROM} {TO} {URGENCY} {PRIORITY} {ID} {TASK} {PARAMS}")
        
        if(TO == "all" or TO == self.botName):
            #check if it is an accept, if it is for this bot accept interruption
            if(TYPE == "accept"):
                self.acceptHandling(TYPE = TYPE, FROM = FROM, TO = TO, URGENCY = URGENCY, PRIORITY = PRIORITY, ID = ID, TASK = TASK, PARAMS = PARAMS)
                        
            #check if its an interruption, if it is handle request
            if(TYPE == "interruption"):
               self.requestHandling(TYPE = TYPE, FROM = FROM, TO = TO, URGENCY = URGENCY, PRIORITY = PRIORITY, ID = ID, TASK = TASK, PARAMS = PARAMS)
            
            #set collision flag and type
            if(TYPE == "collision"):
                self.collisionHandling(TYPE = TYPE, FROM = FROM, TO = TO, URGENCY = URGENCY, PRIORITY = PRIORITY, ID = ID, TASK = TASK, PARAMS = PARAMS)
            
     
    #handles whether or not to send a request(to handle some interruption) and sending the request itself
    def requestHandling(self, TYPE, FROM, TO, URGENCY, PRIORITY, ID, TASK, PARAMS):
        #check who the interruption is for
        NewMsg = String()#new message
        if((TO == self.botName) or (TO == "all")):
            #choose if a request should be made
            if((int(URGENCY) >= 1) or (self.interruptLoad <= self.maxInterruptLoad)):#if the task is high priority
                #find how much distance must be traveled until the task is reached
                #we do this by creating a copy and then adding the new potential task
                with self.SubtaskListLock:
                    copyOfTasks = self.listOfTasks.copy()
                
                copyOfTasks += Tasks.NewTask(TO, URGENCY, PRIORITY, ID, TASK, PARAMS)

                #run optimization
                copyOfTasks = Tasks.orderSubtasks(copyOfTasks, self.currFinalX, self.currFinalY)
                
                time = self.TimeUntilSubtask(copyOfTasks, ID, 0)
                
                #create a request for the interruption, swap to and from
                NewMsg.data = Tasks.createMsgString(TYPE = "request", FROM = self.botName, TO = FROM, URGENCY = "", PRIORITY = "",
                                                    ID = ID, TASK = TASK, PARAMS = [str(time)])
                                                    
                self.publisher_.publish(NewMsg)#actually publish
                self.get_logger().info(NewMsg.data)#print to terminal
            #ignore interruption
            else:
                NewMsg.data = Tasks.createMsgString(TYPE = "ignore", FROM = self.botName, TO = FROM, URGENCY = "", PRIORITY = "",
                                                    ID = ID, TASK = TASK, PARAMS = [])
                self.publisher_.publish(NewMsg)#actually publish
                self.get_logger().info(NewMsg.data)#print to terminal
        
         #if the interruption does not apply to this bot, send ignore
        else:
            NewMsg.data = Tasks.createMsgString(TYPE = "ignore", FROM = self.botName, TO = FROM, URGENCY = "", PRIORITY = "",
                                                ID = ID, TASK = TASK, PARAMS = [])
            self.publisher_.publish(NewMsg)#actually publish
            self.get_logger().info(NewMsg.data)#print to terminal
    
    #handles actually accepting a new task, creates new tasks and optimizes order placement
    def acceptHandling(self, TYPE, FROM, TO, URGENCY, PRIORITY, ID, TASK, PARAMS):
        #check if it is for this bot
        if(TO == self.botName):
            #wait for the top subtask to be popped
            with self.SubtaskListLock:
                self.get_logger().info(f"accepted {self.botName} {ID}")
            
                #create the subtasks, update interruption load
                newTasks = Tasks.NewTask(TO = TO, URGENCY = URGENCY, PRIORITY = PRIORITY,
                                            ID = ID, TASK = TASK, PARAMS = PARAMS)
                #create task tracker and add to list                    
                self.listOfTaskTrackers.append(Tasks.taskTracker(taskID = ID, numOfSubtasks = len(newTasks)))
                
                #log that new task has been recieved
                self.SendLog(subtask = newTasks[0], event = "task_received")
                
                self.interruptLoad += len(newTasks)
                #update list of tasks 
                self.listOfTasks += newTasks
                self.listOfTasks = Tasks.orderSubtasks(self.listOfTasks, self.currFinalX, self.currFinalY)
                
                #set current highest priority
                if(self.currUrgency < self.listOfTasks[0].urgency):
                    self.currUrgency = self.listOfTasks[0].urgency
                
            #DEBUG
            self.printSubtasks()
    
    #handles collision messages from the server, sets collision flag and set type
    #As of the moment this is not finished or being worked on ~Trey 7/11/26
    def collisionHandling(self, TYPE, FROM, TO, URGENCY, PRIORITY, ID, TASK, PARAMS):
        self.collisionType = PARAMS[0]
        

    #checks if there are subtasks available, does the next subtask if so
    def checkSubTask(self):
        # if a task is already being executed, don't start another one
        if self.currSubtask is not None:
            return
        
        # if there is work available, then do it
        if len(self.listOfTasks) > 0:
            if self.idleTimeStart is not None:
                self.idleTime += time.time() - self.idleTimeStart
                self.idleTimeStart = None
                
            self.ControlLoop()
            self.SendAggregateLog()
    
        # otherwise, robot is idle
        else:
            if self.idleTimeStart is None:
                self.idleTimeStart = time.time()
            
            self.get_logger().info("Idle")
    
    #checks higher urgency interruptions and does a portion of the currentSubtask
    #NOTE: does not set currentSubtask unless a higher priority one is found
    def ControlLoop(self):   
        subtaskNotFinished = True
        #get current subtask and output

        with self.SubtaskListLock:
            self.currSubtask = self.listOfTasks.pop(0)
            self.currFinalX = self.currSubtask.finishX
            self.currFinalY = self.currSubtask.finishY
            self.currUrgency = self.currSubtask.urgency
            self.get_logger().info(f"started {self.currSubtask.urgency} {self.currSubtask.subTaskID} of task " + 
                        f"{self.currSubtask.taskID} final({self.currSubtask.finishX},{self.currSubtask.finishY})")
            
            #log subtask start and task start (if it is the first subtask started)
            self.checkForNewTaskStart()
        
        #while the current task is not finished
        while(subtaskNotFinished):
            #check for potential collision
            if(self.collisionType != "nothing"):
                self.avoidCollisions()
            #if(
            #len(self.listOfTasks) > 0 and
            #self.listOfTasks[0].taskID != self.currSubtask.taskID #NOTE: currsubtask is NOT in the list of tasks, and all subtasks of the same task will have the same urgency
            #check for a higher urgency task, 
            elif(
                len(self.listOfTasks) > 0
                and self.listOfTasks[0].taskID != self.currSubtask.taskID
                and self.currSubtask.isMainTask
                and not self.listOfTasks[0].isMainTask
            ):
                #output interrupted message
                self.get_logger().info(f"INTERRUPTED {self.currSubtask.urgency} {self.currSubtask.subTaskID} of task " + 
                        f"{self.currSubtask.taskID} final({self.currSubtask.finishX},{self.currSubtask.finishY})")
                 #send interruption to logger
                self.SendLog(
                     subtask = self.currSubtask, 
                     event = "task_interrupted"
                 )
                 
                with self.SubtaskListLock:
                    #put current task pack into the list
                    self.listOfTasks.append(self.currSubtask)
                    #re-optimize
                    self.listOfTasks = Tasks.orderSubtasks(self.listOfTasks, self.currFinalX, self.currFinalY)
                    #get new higher urgency task
                    self.currSubtask = self.listOfTasks.pop(0)
                    self.currFinalX = self.currSubtask.finishX
                    self.currFinalY = self.currSubtask.finishY
                    self.currUrgency = self.currSubtask.urgency
                    self.get_logger().info(f"started {self.currSubtask.urgency} {self.currSubtask.subTaskID} of task " + 
                        f"{self.currSubtask.taskID} final({self.currSubtask.finishX},{self.currSubtask.finishY})")
                    
                    self.checkForNewTaskStart()

                #do subtask, use subtaskNotFinished to determine if task needs to be continued
                subtaskNotFinished = self.currSubtask.doSubtask(self.BotLocation)
                
            else:
                #do subtask, use subtaskNotFinished to determine if task needs to be continued
                subtaskNotFinished = self.currSubtask.doSubtask(self.BotLocation)
            time.sleep(.3)
        
        #after finished, delete subtask and output message
        if(self.currSubtask.isMainTask == False):#finished user interrupt task
            self.interruptLoad -= self.interruptLoadDecay
        else:#if the subtask is an explore task
            self.numOfExploreTasksComplete += 1
            
                
        self.get_logger().info(f"finished {self.currSubtask.urgency} {self.currSubtask.subTaskID}"
                + f" of task {self.currSubtask.taskID}\nDeferment time: {time.time() -self.currSubtask.creationTime}")
            
        self.UpdateTracker()#update task tracker
        
        self.currSubtask.destroy_node()
        
        self.currSubtask = None # finished current task
        
        # if nothing is waiting, begin idle timing
        if len(self.listOfTasks) == 0:
            if self.idleTimeStart is None:
                self.idleTimeStart = time.time()
    
    #checks if there is a possible collision and acts to avoid it
    def avoidCollisions(self):
        #check for a possible collision
        if(self.collisionType != "nothing"):
            cmd = Twist()
            
            if(self.collisionType == "wait"):
                self.get_logger().info(f"Avoiding collision: wait")
                self.movementPublisher.publish(cmd)
                #time.sleep(5)
                
            elif(self.collisionType == "reverse"):
                cmd.linear.x = -0.3
                cmd.angular.z = 0.05
                self.movementPublisher.publish(cmd)
                self.get_logger().info(f"Avoiding collision: reverse")
                #time.sleep(5)
                
    #handles updating taskTracker/logging if finished
    def UpdateTracker(self):
        for Task in self.listOfTaskTrackers:
            if (Task.taskID == self.currSubtask.taskID):
                Task.numOfSubtasksLeft -= 1

                self.SendLog(subtask = self.currSubtask, event = "subtask_end")
                
                if (Task.numOfSubtasksLeft <= 0):#if this was the final subtask of the task, log the finish
                    self.get_logger().info(f"finished TASK: {Task.taskID} URGENCY: {self.currSubtask.urgency} PRIORITY: {self.currSubtask.priority}")
                    #send to logger
                    self.SendLog(subtask = self.currSubtask, event = "task_end")
                    
                return
        self.get_logger().info((f"ERROR: Could not find TASK: {Task.taskID} URGENCY: {self.currSubtask.urgency}"))
    
    #checks if the current subtask is the first to be done for the task
    def checkForNewTaskStart(self):
        for Task in self.listOfTaskTrackers:
            if (Task.taskID == self.currSubtask.taskID):#when found in listOfTrackers
                if(Task.numOfSubtasksLeft == Task.numOfSubtasks):
                    self.SendLog(subtask = self.currSubtask, event = "task_start")
                self.SendLog(subtask = self.currSubtask, event = "subtask_start")
    
    #send update to logger node
    def SendLog(self, subtask, event: str):
        #set log vals
        logDict = {
                    "ts_unix": float(time.time()),
                    "robot": self.botName,
                    "task_id": subtask.taskID,
                    "subtask_id": subtask.subTaskID,
                    "baseline_task": str(subtask.isMainTask),
                    "event": event,
                    "urgency": subtask.urgency,
                    "priority": subtask.priority,
                    "task_name": subtask.taskName
                    }
        msg = String()
        msg.data = json.dumps(logDict)#turn python dict to json
        self.logPublisher.publish(msg)#send 
    
    #this sends aggregate data to the launcher, some examples are down time or number of 
    def SendAggregateLog(self, event:str ="aggregate"):
        #set log vals
        logDict = {
                    "ts_unix": float(time.time()),
                    "robot": self.botName,
                    "event": event,
                    "idle_time": self.idleTime,
                    "percent_area_explored": (self.numOfExploreTasksComplete / self.numOfExploreTasks), #percentage of explore tasks completed
                    "distance_traveled": self.BotLocation.getTotalTravelDistance()
                    }
                    
        print(
            f"[DEBUG] {self.botName} | "
            f"idle={self.idleTime:.2f} | "
            f"queue={len(self.listOfTasks)} | "
            f"currTask={self.currSubtask.taskID if self.currSubtask else 'None'} | "
            f"currSubtask={'Yes' if self.currSubtask else 'No'} | "
            f"collision={self.collisionType} | "
            f"explore={self.numOfExploreTasksComplete}/{self.numOfExploreTasks}"
        )
        
        msg = String()
        msg.data = json.dumps(logDict)#turn python dict to json
        self.logPublisher.publish(msg)#send 
        

    #approximates the amount of time until a specific subtask is reached
    #replaced "TravelDistanceUntil" func
    def TimeUntilSubtask(self, subtaskList, taskID, subTaskID):
        totalTime = 0.0
        
        #get the pose
        prevX, prevY, prevZ, prevRoll, prevPitch, prevYaw = self.BotLocation.get()

        #get distance between robots current position and the final location of the current subtask
        #time = velocity/distance
        distance = Tasks.findDistance(self.currFinalX, self.currFinalY, prevX, prevY)
        if(distance != 0):
            totalTime += self.currSubtask.linear_speed/distance
        
        totalTime += self.currSubtask.taskTimeRecquired
        prevX, prevY = self.currFinalX, self.currFinalY
        
        #if its not the task we are looking for, find the distance and add it to the total
        for subtask in subtaskList:
            #find travel time from each subtask to the next
            if(subtask.locationBased == True):#only do if the task is location based and if the distance isn't 0
                distance = Tasks.findDistance(self.currFinalX, self.currFinalY, prevX, prevY)
                if(distance != 0):
                    totalTime += self.currSubtask.linear_speed/distance
            
            prevX = subtask.finishX
            prevY = subtask.finishY
            if((subtask.taskID == taskID) and (subtask.subTaskID == subTaskID)):
                return totalTime
            #task time is added after since this func aims to find the time until the task can be STARTED
            #not COMPLETED
            totalTime += subtask.taskTimeRecquired
            
            
    #when getting an update from gazebo, update when 
    def UpdateBotLocation(self, poseMsg):
        self.BotLocation.update(poseMsg)
    
    #predominantly used for debug, prints all of the subtasks that the bot currently has
    def printSubtasks(self):
        #with self.SubtaskListLock:
            self.get_logger().info(f"Current Subtask: TaskName: {self.currSubtask.taskName} TaskID: {self.currSubtask.taskID} SubtaskID: {self.currSubtask.subTaskID} Priority: {self.currSubtask.priority} Urgency: {self.currSubtask.urgency} isMainTask: {str(self.currSubtask.isMainTask)}")
            for subtask in self.listOfTasks:
                self.get_logger().info(f"TaskName: {subtask.taskName} TaskID: {subtask.taskID} SubtaskID: {subtask.subTaskID} Priority: {subtask.priority} Urgency: {subtask.urgency} isMainTask: {str(subtask.isMainTask)}")
       
            
#this object keeps track of the robots odometry, updated inside of botNode
class botLocation():
    def __init__(self):
        #current positon of the robot
        self.X = 0.0
        self.Y = 0.0
        self.Z = 0.0
        
        #rotation
        self.Roll = 0.0
        self.Pitch = 0.0
        self.Yaw = 0.0
        
        #total distance traveled
        self.totalDistanceTraveled = 0.0
        
        #used to check if the BotLocation has recieved its first update
        self.hasInitialLoc = False
        
        #prevent race conditon between reading odom and updating it
        self.odomLock = threading.Lock()
        
        self.distanceLock = threading.Lock()
    
    def update(self, poseMsg):
        newPos = poseMsg.pose.position
        newOrient = poseMsg.pose.orientation
        
        tempRoll = math.atan2(2*(newOrient.w * newOrient.x + newOrient.y*newOrient.z), 1 - 2*(newOrient.x**2 + newOrient.y**2))
        tempPitch = math.asin(2*(newOrient.w *newOrient.y - newOrient.z*newOrient.x))
        tempYaw = math.atan2(2*(newOrient.w*newOrient.z + newOrient.x*newOrient.y), 1 - 2*(newOrient.y**2 + newOrient.z**2))
        
        #update distance. NOTE: since updating botLocation uses an mutually exclusive callback group, there is no race condition for getting self.X and self.Y 
        with self.distanceLock:
            if(self.hasInitialLoc == True):
                #only update if the bot has its initial location
                self.totalDistanceTraveled += Tasks.findDistance(newPos.x, newPos.y, self.X, self.Y)
            else:
                self.hasInitialLoc = True
        
        #use lock to avoid race condition
        with self.odomLock:
            self.X = newPos.x
            self.Y = newPos.y
            self.Z = newPos.z
            
            self.Roll = tempRoll
            self.Pitch = tempPitch
            self.Yaw =  tempYaw
            
    #get the most recent position of the bot
    def get(self):
        with self.odomLock:
            #print(f"{self.X}, {self.Y}, {self.Z}, {self.Roll}, {self.Pitch}, {self.Yaw}")
            return (self.X, self.Y, self.Z, self.Roll, self.Pitch, self.Yaw)
    
    def getTotalTravelDistance(self):
        with self.distanceLock:
            return self.totalDistanceTraveled
    
    def hasInitialLocation(self):
        return self.hasInitialLocation
            
            
 
def main(args=None):

    newBotName = ""
    exploreX = 0.0
    exploreY = 0.0
    #get system argument for bot name
    if(len(sys.argv) < 4):#if there is only one arg then no args were given in cmd
        newBotName = "Generic_Bot"
    else:
        newBotName = sys.argv[1]
        exploreX = float(sys.argv[2])
        exploreY = float(sys.argv[3])
    	
    rclpy.init(args=args)
    
    #create multi threads for the node, 2 for each node
    MultiExecutor = MultiThreadedExecutor(num_threads = 3)

    NewBotNode = botNode(newBotName, exploreX, exploreY)
    
    MultiExecutor.add_node(NewBotNode)
    
    MultiExecutor.spin()

    #rclpy.spin(NewBotNode)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    NewBotNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
