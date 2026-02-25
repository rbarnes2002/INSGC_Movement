import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import copy

import time

import sys
import threading
import Tasks

import json

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
    	
    	#publisher section
        self.publisher_ = self.create_publisher(String, '/BotPublish', 100) 
        
        #publishes to the logging system
        self.logPublisher = self.create_publisher(String, "/task_events", 10)
    	
    	#list of subtasks
        self.listOfTasks = []
    	
    	#data for exploration params
        self.exploreX = explX
        self.exploreY = explY
        self.exploreWidth = 10
        self.exploreHeight = 10
   
    	#initialize exploration
        self.listOfTasks += Tasks.CreateExploreTasks(self.botName, 0, 2, "Explore_"+ self.botName, 
                                                    self.exploreX, self.exploreY, self.exploreWidth, self.exploreHeight)
        
        #task tracker for the main explore task
        self.listOfTaskTrackers = [Tasks.taskTracker(taskID = "Explore_" + self.botName, numOfSubtasks = len(self.listOfTasks))]
                                   
        self.currSubtask = self.listOfTasks.pop(0)#current subtask being done
        #the coords at the end of the current subtask
        self.currFinalX = self.currSubtask.finishX
        self.currFinalY = self.currSubtask.finishY
        self.currUrgency = self.currSubtask.urgency#current highest priority level in the subtask list
    	
    	#user interruption handling, used to check if a bot should request some interruption
        self.interruptLoad = 0.0
        self.maxInterruptLoad = 30.0 #maximum number of user interrupts the robot will have (including current ones)
        self.interruptLoadDecay = 1.0 #each time a user task is done, interruptLoad is decreased by this much
    	
    	#this section is designed to run a check for tasks
        self.SubtaskListLock = threading.Lock()
        self.checkSubTaskTimer = self.create_timer(.5, self.checkSubTask)
        
    #listens to and responds to server
    def botListener(self, msg):
        TYPE, FROM, TO, URGENCY, PRIORITY, ID, TASK, PARAMS = Tasks.ParseMsg(msg.data)
        
        #check if it is an accept, if it is for this bot accept interruption
        if(TYPE == "accept"):
            self.acceptHandling(TYPE = TYPE, FROM = FROM, TO = TO, URGENCY = URGENCY, PRIORITY = PRIORITY, ID = ID, TASK = TASK, PARAMS = PARAMS)
                    
        #check if its an interruption, if it is handle request
        if(TYPE == "interruption"):
           self.requestHandling(TYPE = TYPE, FROM = FROM, TO = TO, URGENCY = URGENCY, PRIORITY = PRIORITY, ID = ID, TASK = TASK, PARAMS = PARAMS)
     
    #handles whether or not to send a request(to handle some interruption) and sending the request itself
    def requestHandling(self, TYPE, FROM, TO, URGENCY, PRIORITY, ID, TASK, PARAMS):
        #msgFields = msg.data.split(" ")
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

    #checks if there are subtasks available, does the next subtask if so
    def checkSubTask(self):
        #if there are tasks 
        if(len(self.listOfTasks) > 0):    
            self.ControlLoop()#do subtask()
        else:
            #No tasks/exploration tasks, create more exploration tasks
            #self.listOfTasks += Tasks.CreateExploreTasks(self.botName, 0, "Explore_"+ self.botName, 
             #                  self.exploreX, self.exploreY, self.exploreWidth, self.exploreHeight)
             self.get_logger().info("No tasks")
    
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
            #check for a higher priority task
            if(self.currUrgency > self.currSubtask.urgency):
                 #output interrupted message
                 self.get_logger().info(f"INTERRUPTED {self.currSubtask.urgency} {self.currSubtask.subTaskID} of task " + 
                        f"{self.currSubtask.taskID} final({self.currSubtask.finishX},{self.currSubtask.finishY})")
                 #send interruption to logger
                 self.SendLog(subtask = self.currSubtask, event = "task_interrupted")
                 
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
            subtaskNotFinished = self.currSubtask.doSubtask()
            time.sleep(.3)
        
        #after finished, delete subtask and output message
        if(self.currSubtask.isMainTask == False):#finished user interrupt task
            self.interruptLoad -= self.interruptLoadDecay
                
        self.get_logger().info(f"finished {self.currSubtask.urgency} {self.currSubtask.subTaskID}"
                + f" of task {self.currSubtask.taskID}\nDeferment time: {time.time() -self.currSubtask.creationTime}")
            
        self.UpdateTracker()#update task tracker
        
        self.currSubtask.destroy_node()
            
                
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
    def SendLog(self, subtask, event):
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

    #approximates the amount of time until a specific subtask is reached
    #replaced "TravelDistanceUntil" func
    def TimeUntilSubtask(self, subtaskList, taskID, subTaskID):
        totalTime = 0.0
        
        prevX, prevY, z, prevYaw, = 0.0, 0.0, 0.0, 0.0
        #get the pose
        while(True):
            pose = Tasks.get_pose(self.botName)
            if(pose is not(None)):
                prevX, prevY, z, prevYaw = pose
                break
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
    MultiExecutor = MultiThreadedExecutor(num_threads = 2)

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
