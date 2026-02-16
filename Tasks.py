import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from geometry_msgs.msg import Twist

import subprocess

import time

import re #used for multi delimiter split
import math

#NOTE FOR ANY NEW KIND OF TASK THE FOLLOWING MUST BE DONE, ALSO now other file than this one should have to be altered
#1) Create relevant subtask classes derived from the subtask class
#2) A create task func must be written, this func creates all of the subtasks and returns a list of them
#3) Said create task func must be added to the NewTask func at the bottom of this file, 
#	This func chooses what create task func to call according to the task argument

######################### HELPER FUNC SECTION  ####################################

#given some parameters, creates a message to send using the old message
#note: parameters (other than receivedMessage) are for the NEW message 
def createMsgString(receivedMessage, TYPE = "", FROM = "", TO = "", URGENCY = "", ID = -1, TASK = ""):
    newMessage = ""
    params = ""# used for task parameters
    receivedMessageFields = receivedMessage.split(" ")
    if(TYPE == ""):
        TYPE = receivedMessageFields[0]
    if(FROM == ""):
        FROM = receivedMessageFields[2]#assuming we are responding to the sender TO and FROM are switched
    if(TO == ""):
        TO = receivedMessageFields[1]
    if(URGENCY == ""):
        URGENCY = receivedMessageFields[3]
    if(ID == -1):
        ID = receivedMessageFields[4]   
    if(TASK == ""):
        TASK = receivedMessageFields[5]
    
    #get params from received message
    i = 6
    while i < len(receivedMessageFields):
        params += " " + receivedMessageFields[i]
        i += 1
        
    newMessage = TYPE + " " + FROM + " " + TO + " " + URGENCY + " " + str(ID) + " " + TASK + params
    return newMessage

#create a response string from the given values
def createMsgString(TYPE, FROM, TO, URGENCY, ID, TASK, PARAMS):

    responseString = ""
    responseString += TYPE + " "
    responseString += FROM + " "
    responseString += TO + " "
    if((TYPE == "interruption") or (TYPE == "accept")):#request and ignore messages dont have urgency
        responseString += URGENCY + " "
    responseString += ID + " "
    responseString += TASK + " "
    for param in PARAMS:
        responseString += param + " "

    return responseString
    
#
#takes a string message received and terms it into variables,this is mainly done for readability
#
def ParseMsg(receivedMessage):
    msgFields = receivedMessage.split()
    
    #if it is a interruption/accept(server) message or a request/ignore(bot) message
    if((msgFields[0] == "interruption") or (msgFields[0] == "accept")):
        TYPE = msgFields[0]
        FROM = msgFields[1]
        TO = msgFields[2]
        URGENCY = msgFields[3]
        TASKID = msgFields[4]
        TASK = msgFields[5]
        PARAMS = []
        #get params from received message
        i = 6
        while i < len(msgFields):
            PARAMS.append(msgFields[i])
            i += 1
        return (TYPE, FROM, TO, URGENCY, TASKID, TASK, PARAMS)
        
    if((msgFields[0] == "request") or (msgFields[0] == "ignore")):
        TYPE = msgFields[0]
        FROM = msgFields[1]
        TO = msgFields[2]
        URGENCY = "-1"#requests and ignore do not have urgency, this is only done to simply addressing 
                    #the output of this func
        TASKID = msgFields[3]
        TASK = msgFields[4]
        PARAMS = []
        #get params from received message
        i = 5
        while i < len(msgFields):
            PARAMS.append(msgFields[i])
            i += 1
        return (TYPE, FROM, TO, URGENCY, TASKID, TASK, PARAMS)
    return None


#helper func to find the distance between two points
def findDistance(x1 = 0.0, y1 = 0.0, x2 = 0.0, y2 = 0.0):
    return ((x2 - x1)**2 + (y2 - y1)**2)**.5
    

#helper func, gets the position of the bot from gazebo
#this func was taken from riley's "move_to_point.py" file
def get_pose(MODEL_NAME):
    """Reads Husky pose from Gazebo using gz model."""
    try:
        out = subprocess.check_output(
            ["gz", "model", "-m", MODEL_NAME, "-p"],
            stderr=subprocess.STDOUT,
            text=True
        ).strip()
        parts = re.split(r'[ ,\[\]]', out)
        if len(parts) != 45:
        #print(f"{MODEL_NAME} {parts}") #debug
            return None
        #x, y, z, roll, pitch, yaw = map(float, parts)
        x, y, z = float(parts[33]), float(parts[34]), float(parts[35])
        roll, pitch, yaw = float(parts[41]), float(parts[42]), float(parts[43])
        return x, y, z, yaw
    except subprocess.CalledProcessError as e:
        #print(f"Could not read pose for {MODEL_NAME}: {e.output.strip()}")
        return None
    
    except FileNotFoundError:
        print("gz command not found")
        return None

######################### SUBTASK SECTION ####################################
#define objects as children of the subtasks class

#
#Parent class for any other kind of subClass
#To create any other kind of subtask doSubtask and getPercentDone MUST be overwritten
#doSubtask MUST return False if it is finished and true if it is 
#look at moveTo for an example 
class SubTask(Node):
    def __init__(self, BotName, SubTaskID, TaskID):
         #give the node a name and a namespace
         super().__init__(str(BotName) + "_" + str(SubTaskID) + "_" + str(TaskID), namespace= f'/model/{BotName}')
         
         self.MovementPublisher = self.create_publisher(Twist, "cmd_vel", 10)
         
         #task descriptions
         self.botName = BotName
         self.taskID = TaskID
         self.subTaskID = SubTaskID
         #priority
         self.urgency = 0
         #whether or not this task is a part of the bots main explore task
         self.isMainTask =  False
         #creation time of the task, used to output deferment time
         self.creationTime = time.time()
         
         #final coordinates after the subtask is finished
         self.finishX = 0.0
         self.finishY = 0.0
         self.finishZ = 0.0
         
         #specific vars for this kind of subtask
         
         
    #this method is needed, this call actually runs the subtask
    def doSubtask(self):
        print("Do the next subtask")
        return False
        
    def getPercentDone(self):
        print("return the percentage done")
        return 0.0

#
#Parent class for any other kind of subClass
#To create any other kind of subtask doSubtask and getPercentDone MUST be overwritten
#look at moveTo for an example 
class SubTask(Node):
    def __init__(self, BotName, SubTaskID, TaskID):
         #give the node a name and a namespace
         super().__init__(str(BotName) + "_" + str(SubTaskID) + "_" + str(TaskID), namespace= f'/model/{BotName}')
         
         self.MovementPublisher = self.create_publisher(Twist, "cmd_vel", 10)
         
         #task descriptions
         self.botName = BotName
         self.taskID = TaskID
         self.subTaskID = SubTaskID
         #priority
         self.urgency = 0
         #whether or not this task is a part of the bots main explore task
         self.isMainTask =  False
         #creation time of the task, used to output deferment time
         self.creationTime = time.time()
         
         #final coordinates after the subtask is finished
         self.finishX = 0.0
         self.finishY = 0.0
         self.finishZ = 0.0
         
         #specific vars for this kind of subtask
         
         
    #this method is needed, this call actually runs the subtask
    def doSubtask(self):
        print("Do the next subtask")
        
    def getPercentDone(self):
        print("return the percentage done")
        return 0.0

#
# Command that simply makes the robot move forward for some number of seconds
#NOTE: AS OF THE 2/16/26 VERSION OF THIS CODE THIS SUBTASK HAS NOT BEEN BUGTESTED
#
class MoveForward(SubTask):

    def __init__(self, BotName, SubTaskID, TaskID, Time):
    
        super().__init__(BotName, SubTaskID, TaskID)
        
         
        #final coordinates after the subtask is finished
        self.finishX = 0.0
        self.finishY = 0.0
        self.finishZ = 0.0
         
        #specific vars for this kind of subtask
        self.time = Time
         
         
    def doSubtask(self):
        #print("Do the next subtask")
        cmd = Twist()
        speed = 3.0
        
        cmd.linear.x = speed
        
        strTime = time.time()
        
        while((time.time() - strTime) <= self.time):
            self.MovementPublisher.publish(cmd)
            time.sleep(.1)
          
          
#
#### MOVE to some specified point in space ####
#
class MoveTo(SubTask):
    def __init__(self, BotName, Urgency, SubTaskID, TaskID, X, Y, Speed):
    
        super().__init__(BotName, SubTaskID, TaskID)
         
        #final coordinates after the subtask is finished
        self.finishX = X
        self.finishY = Y
        self.finishZ = 0.0
        
        self.urgency = Urgency
        
        #specific vars for this kind of subtask
        self.linear_speed = float(Speed)
        self.angular_speed = .2#float(Speed)
        self.distance_tolerance = 1.0
    
    #much of this code was taken from Riley's "move_to_point.py" func "control_loop"
    def doSubtask(self):
            pose = get_pose(self.botName)
            if pose is None:
                #self.get_logger().warn("Could not read Husky pose.")
                #return
                self.MovementPublisher.publish(Twist())
                return True

            x, y, z, yaw = pose
            
            # Distance to target
            dx = self.finishX - x
            dy = self.finishY - y
            distance = math.sqrt(dx*dx + dy*dy)

            # Stop if close enough
            if distance < self.distance_tolerance:
                #self.stop_robot()
                #self.get_logger().info("Reached target point!")
                #rclpy.shutdown()
                return False
                
            #convert yaw to a positive radian #note: gazebos yaw treats CLOCKWISE as positive,
            # hence we must flip
            yaw = self.normalize_angle(yaw - math.pi)

            # Compute heading angle
            target_angle = self.normalize_angle(math.atan2(dy, dx))
        
            angle_error = target_angle - yaw

            cmd = Twist()

            # Rotate toward target
            if abs(angle_error) > 0.1:
                direction = 1
                if(target_angle > yaw):
                    if(abs(angle_error) > math.pi):
                        direction = 1
                    else:
                        direction = -1
                else:
                    if(abs(angle_error) > math.pi):
                        direction = -1
                    else:
                        direction = 1
                
                cmd.angular.z = self.angular_speed * direction
                cmd.linear.x = .1
            else:
                # Move forward when roughly facing target
                cmd.linear.x = self.linear_speed
            
            #print(f"HERE {cmd.angular.z} {cmd.linear.x}")
            #had to change publisher ~trey
            self.MovementPublisher.publish(cmd)
            return True
            
    #helper method
    def normalize_angle(self, angle):
        """Normalize angle to [0, 2pi]."""
        while(not(angle > 0) and (angle < 2*math.pi)):
            if(angle < 0):
                angle += 2*math.pi
            if(angle > 2*math.pi):
                angle -= 2*math.pi
        return angle

######################### CREATE TASK SECTION ####################################
#creates a list of subtasks in order to fufill the given task   


#
# Creates a list of moveForward subtasks
#
def CreateMoveForwardTask(BotName, taskID, speed = 3.0, time = 10):
        subTaskList = []
        
        for i in range(time):
            #each subtask will be moving forward for 1 sec
            currSubTask = MoveForward(BotName, str(i), taskID, 1)
            subTaskList.append(currSubTask)
            
        return subTaskList

#
# Creates a single moveto subtask and returns it wrapped as a list
#
def CreateMoveToTask(BotName, urgency, taskID, X = 0, Y = 0, speed = 1.0):
    subTaskList = [MoveTo(BotName, urgency, 0, taskID, X, Y, speed)]
    return (subTaskList)
    
#
# Create a series of moveto tasks that explore a grid area
#
# creates a "Z" pattern of exploring some area
# * * * * * * * * 
# *
# * * * * * * * *
#               *   Height
# * * * * * * * * 
# *              
# * * * * * * * * (X_origin, Y_Origin)
#      Width
def CreateExploreTasks(BotName, urgency, taskID, X_Origin, Y_Origin, X_Width, Y_Height, speed = .5):
    subTaskList = []
    
    increment = 1 #space between exploration points
    
    direction = 1 #current moving direction in the x
    
    subtaskID = 0
    
    #these are coords relative to the X and Y origin of the task 
    X = 0
    Y = 0
    
    while(Y <= Y_Height):
        while(True): #inner X loop is set up like a do while loop
            newSubTask = MoveTo(BotName, urgency, subtaskID, taskID, X + X_Origin, Y + Y_Origin, speed)
            newSubTask.isMainTask = True
            subTaskList.append(newSubTask)
            subtaskID += 1
            #move the nextSubtask one increment in the current direction
            X += increment*direction
            if((abs(X) >= X_Width) or (abs(X) < 1)):
                break
           
        #we "skip" a row here
        Y += increment
        newSubTask = MoveTo(BotName, urgency, subtaskID, taskID, X + X_Origin, Y + Y_Origin, speed)
        newSubTask.isMainTask = True
        subTaskList.append(newSubTask)
        
        subtaskID += 1
        #swap the moving X direction
        direction = direction * -1
        Y += increment#increment again for the new actual row
        
    return subTaskList
    

######################### NewTask ####################################
#ALL NEW TASKS MUST BE ADDED TO THIS FUNC
#THIS FUNC CHOOSES WHICH TASK TO CREATE BASED ON THE TASK ARGUMENT
def NewTask(TO, URGENCY, ID, TASK, PARAMS):
    #Note: my version of python doesnt have match cases~Trey
    if(TASK == "moveto"):
        return CreateMoveToTask(TO, int(URGENCY), ID, int(PARAMS[0]), int(PARAMS[1]), int(PARAMS[2]))
    elif(TASK == "exploreArea"):
        return CreateExploreTasks(TO, int(URGENCY), ID, int(PARAMS[0]), int(PARAMS[1]),
                                    int(PARAMS[2]), int(PARAMS[3]), float(PARAMS[4]))
              
#this func optimizes the current subtasks, this is basically the traveling salesmen problem, uses nearest neighbor
def OptimizeSubTasks(SubTaskList, currFinalX, currFinalY):
    
    #number of nodes
    dim = len(SubTaskList) + 1
    
    #number of visited nodes
    numOfVisited = 0
    
    #eventually, this will be time, but is distance as of ~1/14/2026 trey
    totalTime = 0
        
    #current sub task that we are finding the nearest neighbor for
    currSubtaskIndex = 0
    
    #leaving it for now, but may update this to just swap around subtasklist later
    #find nearest neighbor that isnt taken
    while(numOfVisited < dim -1):
        nearestNeigh = -1
        
        currDistance =  9e7
        
        #get current subtask coords
        x1, y1 = currFinalX, currFinalY
        if(currSubtaskIndex != 0):
                x1 = SubTaskList[currSubtaskIndex -1].finishX
                y1 = SubTaskList[currSubtaskIndex -1].finishY
        
        #look through neighbors
        for adjSubtaskIndex in range(numOfVisited, dim):
            if (currSubtaskIndex == adjSubtaskIndex):
                continue
            #find distance
            #since task currently being PERFORMED will NOT be in subtasklist
            x2, y2 = currFinalX, currFinalY
            if(adjSubtaskIndex != 0):
                x2 = SubTaskList[adjSubtaskIndex -1].finishX
                y2 = SubTaskList[adjSubtaskIndex -1].finishY
            
            #find dis to neighbor
            tempDistance = findDistance(x1, y1, x2, y2)
            
            #find the angle between the points, ~later iteration(1/14/25)
            
            #if new nearest neighbor is found
            if((tempDistance < currDistance)):
                #update distance
                currDistance = tempDistance
                nearestNeigh = adjSubtaskIndex
        
        #swap nearest neighbor with the next index
        SubTaskList[currSubtaskIndex], SubTaskList[nearestNeigh -1] = SubTaskList[nearestNeigh -1], SubTaskList[currSubtaskIndex]
        
        #add time
        totalTime += currDistance 
        
        currSubtaskIndex += 1
        numOfVisited += 1
        #debug
        #for subtask in SubTaskList:
            #print(subtask.subTaskID)
            
    return totalTime

#orders subtasks based on priority AND optimization
def orderSubtasks(SubTaskList, currFinalX, currFinalY):
    
    orderedSubtaskList = []
    
    for urgency in range(10, -1, -1):#for level of priority
        tempList = []#holds all of the subtasks of the current priority
        #for each subtask
        for subtask in SubTaskList:
            if (subtask.urgency == urgency):#add if found
                tempList.append(subtask)
        
        if(len(tempList) != 0):
            #optimize temp list and push it onto the ordered list
            OptimizeSubTasks(tempList, currFinalX, currFinalY)
            orderedSubtaskList += tempList
            
            #update currFinal coords
            currFinalX = tempList[-1].finishX
            currFinalY = tempList[-1].finishY
            
    return orderedSubtaskList
            
    
