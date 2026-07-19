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
def createMsgString(receivedMessage, TYPE = "", FROM = "", TO = "", URGENCY = "", PRIORITY ="", ID = -1, TASK = ""):
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
    if(PRIORITY == ""):
        PRIORITY = receivedMessageFields[4]
    if(ID == -1):
        ID = receivedMessageFields[5]   
    if(TASK == ""):
        TASK = receivedMessageFields[6]
    
    #get params from received message
    i = 7
    while i < len(receivedMessageFields):
        params += " " + receivedMessageFields[i]
        i += 1
        
    newMessage = TYPE + " " + FROM + " " + TO + " " + URGENCY + " " + str(ID) + " " + TASK + params
    return newMessage

#create a response string from the given values
def createMsgString(TYPE, FROM, TO, URGENCY, PRIORITY, ID, TASK, PARAMS):

    responseString = ""
    responseString += TYPE + " "
    responseString += FROM + " "
    responseString += TO + " "
    if((TYPE == "interruption") or (TYPE == "accept") or (TYPE == "collision")):#request and ignore messages dont have urgency
        responseString += URGENCY + " "
        responseString += PRIORITY + " "
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
    if((msgFields[0] == "interruption") or (msgFields[0] == "accept") or (msgFields[0] == "collision")):
        TYPE = msgFields[0]
        FROM = msgFields[1]
        TO = msgFields[2]
        URGENCY = msgFields[3]
        PRIORITY = msgFields[4]
        TASKID = msgFields[5]
        TASK = msgFields[6]
        PARAMS = []
        #get params from received message
        i = 7
        while i < len(msgFields):
            PARAMS.append(msgFields[i])
            i += 1
        return (TYPE, FROM, TO, URGENCY, PRIORITY, TASKID, TASK, PARAMS)
    
    #treated differently since requests and ignore do not have priority or urgency 
    if((msgFields[0] == "request") or (msgFields[0] == "ignore")):
        TYPE = msgFields[0]
        FROM = msgFields[1]
        TO = msgFields[2]
        URGENCY = "-1"#requests and ignore do not have urgency/priority, this is only done to simply addressing 
                    #the output of this func
        PRIORITY = "-1"
        TASKID = msgFields[3]
        TASK = msgFields[4]
        PARAMS = []
        #get params from received message
        i = 5
        while i < len(msgFields):
            PARAMS.append(msgFields[i])
            i += 1
        return (TYPE, FROM, TO, URGENCY, PRIORITY, TASKID, TASK, PARAMS)
        
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
            print(f"{MODEL_NAME} {parts}") #debug
            print("Gazebo No RESPOND")
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
        
#gurantees a pose is returned
def getPoseHelper(MODEL_NAME):
    while(True):
        pose = get_pose(MODEL_NAME)
        if(pose != None):
             return pose

   
######################### SUBTASK SECTION ####################################
#define objects as children of the subtasks class

#used to keep track data for a TASK not just a subtask
class taskTracker:
    def __init__(self, taskID, numOfSubtasks):
        self.taskID = taskID
        self.numOfSubtasks = numOfSubtasks
        self.numOfSubtasksLeft = self.numOfSubtasks

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
         self.taskName = "GenericTaskName"
         #urgency/priority
         self.priority = 0
         self.urgency = 0
         #whether or not this task is a part of the bots main explore task
         self.isMainTask =  False
         #creation time of the task, used to output deferment time
         self.creationTime = time.time()
         
         #final coordinates after the subtask is finished
         self.finishX = 0.0
         self.finishY = 0.0
         self.finishZ = 0.0
         self.locationBased = True
         
         #Movement vars
         self.linear_speed = 0.0
         self.angular_speed = 0.0
         
         #Time needed for the task itself, does NOT include travel time to subtask location
         #this is useful for things like Wait/moveForward commands
         #for moveto commands this is 0 since the entire command is just moving to the location
         self.taskTimeRecquired = 0.0
         
         #specific vars for this kind of subtask
         
         
    #this method is needed, this call actually runs the subtask
    def doSubtask(self, botLocObj = None):
        print("Do the next subtask")
        return False
        
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
# subtask to wait for some amount of time
# The total time is waited in intervals of timeChunkSize
#
class Wait(SubTask):
    def __init__(self, BotName, Urgency, Priority, SubTaskID, TaskID, waitTime, timeChunk = 3.0):
    
        super().__init__(BotName, SubTaskID, TaskID)
        #change descriptions
        self.taskName = "Wait"
         
        #final coordinates after the subtask is finished
        self.finishX = 0.0
        self.finishY = 0.0
        self.finishZ = 0.0
        
        self.angular_speed = 1.0
        
        self.urgency = Urgency
        self.priority = Priority
        
        self.isLocationBased = False
        
        #vars for this specific kind of subtask
        self.timeWaited = 0.0#time that has already been waited
        self.taskTimeRecquired = waitTime#total time to be waited
        self.timeChunkSize = timeChunk#size of each chunk of time to wait
    
    def doSubtask(self, botLocObj = None):
        cmd = Twist()
        cmd.angular.z = self.angular_speed
        #wait in intervals f timezChunkSize, if time left is less than chunkSize then wait that amount of time
        if(self.timeChunkSize <= (self.taskTimeRecquired - self.timeWaited)):
            self.MovementPublisher.publish(cmd)
            time.sleep(self.timeChunkSize)
            self.timeWaited += self.timeChunkSize
            return True
        else:#when waiting last chunk of time
            self.MovementPublisher.publish(cmd)
            time.sleep(self.taskTimeRecquired - self.timeChunkSize)
            return False
            
#
#### MOVE to some specified point in space ####
#
class MoveTo(SubTask):
    def __init__(self, BotName, Urgency, Priority, SubTaskID, TaskID, X, Y, Speed):
    
        super().__init__(BotName, SubTaskID, TaskID)
        #change descriptions
        self.taskName = "MoveTo"
         
        #final coordinates after the subtask is finished
        self.finishX = X
        self.finishY = Y
        self.finishZ = 0.0
        
        self.urgency = Urgency
        self.priority = Priority
        
        #specific vars for this kind of subtask
        self.linear_speed = Speed
        self.angular_speed = .2#float(Speed)
        self.distance_tolerance = 1.0
    
    #much of this code was taken from Riley's "move_to_point.py" func "control_loop"
    def doSubtask(self, botLocObj = None):
            
            x = 0.0
            y = 0.0
            z = 0.0 
            yaw = 0.0
            roll = 0.0
            pitch = 0.0
            #botLocObj = None
            
            if(botLocObj != None):
                x, y, z, roll, pitch, yaw = botLocObj.get()
            else:
                pose = getPoseHelper(self.botName)
                x, y, z, yaw = pose
            
            # Distance to target
            dx = self.finishX - x
            dy = self.finishY - y
            distance = math.sqrt(dx*dx + dy*dy)

            # Stop if close enough
            if distance < self.distance_tolerance:
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



#tell the robot to move in a cardinal direction for some amount of time
class MoveCardinalDirection(SubTask):
    def __init__(self, BotName, Urgency, Priority, SubTaskID, TaskID, direction = "North", time = 5.0):
    
        super().__init__(BotName, SubTaskID, TaskID)
        #change descriptions
        self.taskName = "Move_" + direction
         
        #final coordinates after the subtask is finished
        self.finishX = 0.0
        self.finishY = 0.0
        self.finishZ = 0.0
        
        self.linear_speed = 0.5
        self.angular_speed = 0.2
        
        self.urgency = Urgency
        self.priority = Priority
        
        self.isLocationBased = False
        
        #vars for this specific kind of subtask
        direction = direction.lower()
        #the direction in radians that the robot will be travelling in 
        self.direction: float = 0.0
        
        if(direction == "north"):
            self.direction = 0.0
        elif(direction == "south"):
            self.direction = math.pi
        elif(direction == "east"):
            self.direction = 1.5*math.pi
        elif(direction == "west"):
            self.direction = .5*math.pi
        else:
            print("incorrect direction")
            
        self.time = time
    
    def doSubtask(self, botLocObj = None):
        x = 0.0
        y = 0.0
        z = 0.0 
        yaw = 0.0
        roll = 0.0
        pitch = 0.0
        #botLocObj = None
            
        if(botLocObj != None):
            x, y, z, roll, pitch, yaw = botLocObj.get()
        else:
            pose = getPoseHelper(self.botName)
            x, y, z, yaw = pose
        
        #convert yaw to a positive radian #note: gazebos yaw treats CLOCKWISE as positive,
        # hence we must flip
        yaw = self.normalize_angle(yaw - math.pi)
        
        angle_error = self.direction - yaw
        
        cmd = Twist()

        # Rotate toward target
        if abs(angle_error) > 0.1:
            direction = 1
            if(self.direction > yaw):
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
            self.MovementPublisher.publish(cmd)
            return True
        else:
            # Move forward when facing cardinal direction
            cmd.linear.x = self.linear_speed
            self.MovementPublisher.publish(cmd)
            time.sleep(self.time)
            
            #finish subtask
            return False
        
        
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
def CreateMoveToTask(BotName, urgency, priority, taskID, X = 0, Y = 0, speed = 1.0):
    subTaskList = [MoveTo(BotName, urgency, priority, 0, taskID, X, Y, speed)]
    return(subTaskList)


#
#Create a single wait subtask, returns as a list of a single subtask
#
def CreateWaitTask(BotName, urgency, priority, taskID, waitTime, timeChunkSize = 3.0):
    subTaskList = [Wait(BotName, urgency, priority, 0, taskID, waitTime, timeChunk = 3.0)]
    return(subTaskList)
    
#, int(URGENCY), int(PRIORITY), ID, int(PARAMS[0]), int(PARAMS[1])
# Create a series of moveto tasks that explore a grid area
#
# creates a "Z" pattern of exploring some area
# * * * * * * * * 
# *
# * * * * * * * *
#                 *   Height
# * * * * * * * * 
# *              
# * * * * * * * * (X_origin, Y_Origin)
#      Width
def CreateExploreTasks(BotName, urgency, priority, taskID, X_Origin, Y_Origin, X_Width, Y_Height, speed = .5):
    subTaskList = []
    
    increment = 1 #space between exploration points
    
    direction = 1 #current moving direction in the x
    
    subtaskID = 0
    
    #these are coords relative to the X and Y origin of the task 
    X = 0
    Y = 0
    
    while(Y <= Y_Height):
        while(True): #inner X loop is set up like a do while loop
            newSubTask = MoveTo(BotName, urgency, priority, subtaskID, taskID, X + X_Origin, Y + Y_Origin, speed)
            newSubTask.isMainTask = True
            newSubTask.taskName = "Explore"
            subTaskList.append(newSubTask)
            subtaskID += 1
            #move the nextSubtask one increment in the current direction
            X += increment*direction
            if((abs(X) >= X_Width) or (abs(X) < 1)):
                break
           
        #we "skip" a row here
        Y += increment
        newSubTask = MoveTo(BotName, urgency, priority, subtaskID, taskID, X + X_Origin, Y + Y_Origin, speed)
        newSubTask.isMainTask = True
        subTaskList.append(newSubTask)
        
        subtaskID += 1
        #swap the moving X direction
        direction = direction * -1
        Y += increment#increment again for the new actual row
        
    return subTaskList
    
#
#Creates a single MoveCardinalDirectionTask and returns it wrapped as a list
#
def CreateMoveCardinalDirectionTask(BotName, urgency, priority, taskID, direction, time):
    subTaskList = [MoveCardinalDirection(BotName, urgency, priority, 0, taskID, direction, time)]
    return (subTaskList)
    

######################### NewTask ####################################
#ALL NEW TASKS MUST BE ADDED TO THIS FUNC
#THIS FUNC CHOOSES WHICH TASK TO CREATE BASED ON THE TASK ARGUMENT
def NewTask(TO, URGENCY, PRIORITY, ID, TASK, PARAMS):
    #Note: my version of python doesnt have match cases~Trey
    if(TASK == "moveto"):
        return CreateMoveToTask(TO, int(URGENCY), int(PRIORITY), ID, int(PARAMS[0]), int(PARAMS[1]), float(PARAMS[2]))
    elif(TASK == "exploreArea"):
        return CreateExploreTasks(TO, int(URGENCY), int(PRIORITY), ID, int(PARAMS[0]), int(PARAMS[1]),
                                    int(PARAMS[2]), int(PARAMS[3]), float(PARAMS[4]))
    elif(TASK == "wait"):
        return CreateWaitTask(TO, int(URGENCY), int(PRIORITY), ID, float(PARAMS[0]), float(PARAMS[1]))
    elif(TASK == "moveCardinalDirection"):
        print(f"time is {PARAMS[1]}")
        return CreateMoveCardinalDirectionTask(TO, int(URGENCY), int(PRIORITY), ID, PARAMS[0], float(PARAMS[1]))
              
#this func optimizes the current subtasks, this is basically the traveling salesmen problem, uses nearest neighbor
#optimizes the array given, helper func orderSubtasks
#optimizes in terms of TIME
def OptimizeSubTasks(SubTaskList, currFinalX, currFinalY):
    
    #number of nodes
    dim = len(SubTaskList) + 1
    
    #number of visited nodes
    numOfVisited = 0
        
    #current sub task that we are finding the nearest neighbor for
    currSubtaskIndex = 0
    
    totalTime = 0.0
    
    #leaving it for now, but may update this to just swap around subtasklist later
    #find nearest neighbor that isnt taken
    while(numOfVisited < dim -1):
        #initialize for this run
        nearestNeigh = -1
        currTime =  9e7
        
        
        tempTime = 0.0
        
        #get current subtask coords
        x1, y1 = currFinalX, currFinalY
        if(currSubtaskIndex != 0):
                x1 = SubTaskList[currSubtaskIndex -1].finishX
                y1 = SubTaskList[currSubtaskIndex -1].finishY
        
        #look through neighbors
        for adjSubtaskIndex in range(numOfVisited, dim):
            tempTime = 0.0
            if (currSubtaskIndex == adjSubtaskIndex):
                continue
            #find location of adjacent task
            #since task currently being PERFORMED will NOT be in subtasklist
            x2, y2 = currFinalX, currFinalY
            if(adjSubtaskIndex != 0):
                x2 = SubTaskList[adjSubtaskIndex -1].finishX
                y2 = SubTaskList[adjSubtaskIndex -1].finishY
            
            #if task is location based, find traveltime between currSubtask and adjacent task
            if(SubTaskList[adjSubtaskIndex -1].locationBased == True):
                #find dist to neighbor
                tempDistance = findDistance(x1, y1, x2, y2)
                if(tempDistance != 0):
                    tempTime = SubTaskList[adjSubtaskIndex -1].linear_speed/(tempDistance)
            
            #add extra task time
            tempTime += SubTaskList[adjSubtaskIndex -1].taskTimeRecquired
            
            #if new nearest neighbor is found
            if((tempTime < currTime)):
                #update time
                currTime = tempTime
                nearestNeigh = adjSubtaskIndex
        
        #swap nearest neighbor with the next index
        SubTaskList[currSubtaskIndex], SubTaskList[nearestNeigh -1] = SubTaskList[nearestNeigh -1], SubTaskList[currSubtaskIndex]
        
        #add time
        totalTime += currTime
        
        currSubtaskIndex += 1
        numOfVisited += 1
        #debug
            
    return totalTime

#orders subtasks based on Urgency, then priority, then optimization
def orderSubtasks(SubTaskList, currFinalX, currFinalY):
    
    orderedSubtaskList = []
    
    for urgency in range(3, -1, -1):#for level of priority
        for priority in range(10, -1, -1):
    
            tempList = []#holds all of the subtasks of the current priority
            #for each subtask/collect subtasks that are of the current urgency and priority
            for subtask in SubTaskList:
                if ((subtask.urgency == urgency) and (subtask.priority == priority)):#add if found
                    tempList.append(subtask)
                    
            #optimize temp list and push it onto the ordered list        
            if(len(tempList) != 0):
                OptimizeSubTasks(tempList, currFinalX, currFinalY)
                orderedSubtaskList += tempList
                
                #update currFinal coords
                currFinalX = tempList[-1].finishX
                currFinalY = tempList[-1].finishY
            
    return orderedSubtaskList
            
    
