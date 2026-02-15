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
    	
    	#list of tasks
        self.listOfTasks = []
    	
    	#data for exploration params
        self.exploreX = explX
        self.exploreY = explY
        self.exploreWidth = 2
        self.exploreHeight = 2
   
    	#initialize exploration
        self.listOfTasks += Tasks.CreateExploreTasks(self.botName, 0, "Explore_"+ self.botName, 
                                                    self.exploreX, self.exploreY, self.exploreWidth, self.exploreHeight)
                                   
        self.currSubtask = self.listOfTasks.pop(0)#current subtask being done
        #the coords at the end of the current subtask
        self.currFinalX = self.currSubtask.finishX
        self.currFinalY = self.currSubtask.finishY
        self.currUrgency = self.currSubtask.urgency#current highest priority level in the subtask list
    	
    	#user interruption handling, used to check if a bot should request some interruption
        self.interruptLoad = 0.0
        self.maxInterruptLoad = 2.0 #maximum number of user interrupts the robot will have(including current ones)
        self.interruptLoadDecay = 1.0 #each time a user task is done, interruptLoad is decreased by this much
    	
    	#this section is designed to run a check for tasks
        self.SubtaskListLock = threading.Lock()
        self.checkSubTaskTimer = self.create_timer(.5, self.checkSubTask)
        
    #listens to and responds to server
    def botListener(self, msg):
        msgFields = msg.data.split(" ")
        TYPE, FROM, TO, URGENCY, ID, TASK, PARAMS = Tasks.ParseMsg(msg.data)
        #print(f"{TYPE} {FROM} {TO} {URGENCY} {ID} {TASK}")
        #print(PARAMS)
        
        #check if it is an accept, if it is for this bot accept interruption
        if(TYPE == "accept"):
            self.acceptHandling(TYPE = TYPE, FROM = FROM, TO = TO, URGENCY = URGENCY, ID = ID, TASK = TASK, PARAMS = PARAMS)
                    
        #check if its an interruption, if it is handle request
        if(TYPE == "interruption"):
           self.requestHandling(TYPE = TYPE, FROM = FROM, TO = TO, URGENCY = URGENCY, ID = ID, TASK = TASK, PARAMS = PARAMS)
     
    #handles whether or not to send a request(to handle some interruption) and sending the request itself
    def requestHandling(self, TYPE, FROM, TO, URGENCY, ID, TASK, PARAMS):
        #msgFields = msg.data.split(" ")
        #check who the interruption is for
        if((TO == self.botName) or (TO == "all")):
            NewMsg = String()#new message
            #choose if a request should be made
            if(self.interruptLoad <= self.maxInterruptLoad):
                #find how much distance must be traveled until the task is reached
                #we do this by creating a copy and then adding the new potential task
                with self.SubtaskListLock:
                    copyOfTasks = self.listOfTasks.copy()
                    
                copyOfTasks += Tasks.NewTask(TO, URGENCY, ID, TASK, PARAMS)
                #run optimization here?
                distance = self.TravelDistanceUntil(copyOfTasks, ID, 0)
               
                #create a request for the interruption, swap to and from
                NewMsg.data = Tasks.createMsgString(TYPE = "request", FROM = self.botName, TO = FROM, URGENCY = "",
                                                    ID = ID, TASK = TASK, PARAMS = [str(distance)])
            #ignore interruption
            else:
                NewMsg.data = Tasks.createMsgString(TYPE = "ignore", FROM = self.botName, TO = FROM, URGENCY = "",
                                                    ID = ID, TASK = TASK, PARAMS = [])
                                                    
            self.publisher_.publish(NewMsg)#actually publish
            self.get_logger().info(NewMsg.data)#print to terminal
    
    #handles actually accepting a new task, creates new tasks and optimizes order placement
    def acceptHandling(self, TYPE, FROM, TO, URGENCY, ID, TASK, PARAMS):
        #check if it is for this bot
        if(TO == self.botName):
            #wait for the top subtask to be popped
            with self.SubtaskListLock:
                self.get_logger().info(f"accepted {self.botName} {ID}")
            
                #create the subtasks, update interruption load
                newTasks = Tasks.NewTask(TO = TO, URGENCY = URGENCY,
                                            ID = ID, TASK = TASK, PARAMS = PARAMS)
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
            self.listOfTasks += Tasks.CreateExploreTasks(self.botName, 0, "Explore_"+ self.botName, 
                               self.exploreX, self.exploreY, self.exploreWidth, self.exploreHeight)
    
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
            self.get_logger().info(f"{self.botName} started {self.currSubtask.subTaskID} of task" + 
                        f"{self.currSubtask.taskID} final({self.currSubtask.finishX},{self.currSubtask.finishY})") 
                        
        while(subtaskNotFinished):
            #check for a higher priority task
            if(self.currUrgency > self.currSubtask.urgency):
                 #output interrupted message
                 self.get_logger().info(f"{self.botName} INTERRUPTED {self.currSubtask.subTaskID} of task " + 
                        f"{self.currSubtask.taskID} final({self.currSubtask.finishX},{self.currSubtask.finishY})")
                        
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
                    self.get_logger().info(f"{self.botName} started {self.currSubtask.subTaskID} of task " + 
                        f"{self.currSubtask.taskID} final({self.currSubtask.finishX},{self.currSubtask.finishY})")
            
            #do subtask
            subtaskNotFinished = self.currSubtask.doSubtask()
            time.sleep(.3)
        
        #after finished, delete subtask and output message
        if(self.currSubtask.isMainTask == False):#finished user interrupt task
            self.interruptLoad -= self.interruptLoadDecay
                
        self.get_logger().info(f"{self.botName} finished subtask {self.currSubtask.subTaskID}"
                + f" of task {self.currSubtask.taskID}\nDeferment time: {time.time() -self.currSubtask.creationTime}")
        self.currSubtask.destroy_node()
            
                
            

    #find how much distance will be covered until a specific subtask is reached
    def TravelDistanceUntil(self, taskList, taskID, subTaskID):
        totalDistance = 0.0
        
        prevX, prevY, z, prevYaw, = 0.0, 0.0, 0.0, 0.0
        #get the pose
        while(True):
            pose = Tasks.get_pose(self.botName)
            if(pose is not(None)):
                prevX, prevY, z, prevYaw = pose
                break
        #get distance between robots current position and the final location of the current subtask
        totalDistance = Tasks.findDistance(self.currFinalX, self.currFinalY, prevX, prevY)
        prevX, prevY = self.currFinalX, self.currFinalY
        
        #if its not the task we are looking for, find the distance and add it to the total
        for subtask in taskList:
            totalDistance += Tasks.findDistance(subtask.finishX, subtask.finishY, prevX, prevY)
            prevX = subtask.finishX
            prevY = subtask.finishY
            if((subtask.taskID == taskID) and (subtask.subTaskID == subTaskID)):
                return totalDistance
        
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
