import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from Tasks import createMsgString
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PoseStamped
import math

import threading
import Tasks
import sys
import time


class ServerNode(Node):

    def __init__(self, numOfBots):
        super().__init__('server_1')
        
        #publisher section
        self.publisher_ = self.create_publisher(String, '/ServerPublish', 100) 
        
        #timer_period = 5.0  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.InterruptID = 0
        #self.i = 0
        
        #number of bots in the simulation
        self.numOfBots = numOfBots
        
        #number of interrupts that have been sent to the bots and have NOT been given to a bot
        self.numActiveInterrupts= 0
        self.maxActiveInterrupts = 1#max number of active interrupts 
        
        #subsriber section, listens to the bots
        self.subscription_1 = self.create_subscription(String, '/BotPublish', self.serverListener, 100)
        
        #listens to the user
        self.subscription_2 = self.create_subscription(String, 'userTopic', self.addressUserInput, 100)
        
        #minimum Distance to detect a collision
        self.collisionDist = 3.0
        
        #botPositions
        self.botPositions = []
        
        #determines if a collision was just successfully avoided
        self.avoidedCollision = False
        
        #which bots have been told change course due to collision
        
        #bot POSE listener section
        #quality of service profile
        qos_Profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                                history=HistoryPolicy.KEEP_LAST,
                                                depth = 10,
                                                lifespan = Duration(seconds = 10))
        
        self.poseCallBackGroup = MutuallyExclusiveCallbackGroup()
        
        #an array of bot position listeners 
        self.botPoseSubscription = []
        #an array of bot 
        if(self.numOfBots >= 1):
            self.botPositions.append((0,0,0,0,0,0))
            self.botPoseSubscription.append(self.create_subscription(PoseStamped, 
                                                                               f"/model/robot1/pose",
                                                                               self.robot1Loc,
                                                                               qos_Profile,
                                                                               callback_group = self.poseCallBackGroup))
            if(self.numOfBots >= 2):
                self.botPositions.append((0,0,0,0,0,0))
                self.botPoseSubscription.append(self.create_subscription(PoseStamped, 
                                                                                   f"/model/robot2/pose",
                                                                                   self.robot2Loc,
                                                                                   qos_Profile,
                                                                                   callback_group = self.poseCallBackGroup))
                if(self.numOfBots >= 3):
                    self.botPositions.append((0,0,0,0,0,0))
                    self.botPoseSubscription.append(self.create_subscription(PoseStamped, 
                                                                               f"/model/robot3/pose",
                                                                               self.robot3Loc,
                                                                               qos_Profile,
                                                                               callback_group = self.poseCallBackGroup))
                    if(self.numOfBots >= 4):
                        self.botPositions.append((0,0,0,0,0,0))
                        self.botPoseSubscription.append(self.create_subscription(PoseStamped, 
                                                                                           f"/model/robot4/pose",
                                                                                           self.robot4Loc,
                                                                                           qos_Profile,
                                                                                           callback_group = self.poseCallBackGroup))
                        if(self.numOfBots == 5):
                            self.botPositions.append((0,0,0,0,0,0))
                            self.botPoseSubscription.append(self.create_subscription(PoseStamped, 
                                                                                                f"/model/robot5/pose",
                                                                                                self.robot5Loc,
                                                                                                qos_Profile,
                                                                                                callback_group = self.poseCallBackGroup))
                             
     
        #holds all interruptions not yet given out to the bots
        self.listOfInterrupts = []
        
        
    #listens and publishes, responds to bot requests
    def serverListener(self, msg):
        #parse msg string, note when hearing a request the params are [distance]
        TYPE, FROM, TO, URGENCY, PRIORITY, ID, TASK, PARAMS = Tasks.ParseMsg(msg.data)
        
        #check if interruption is still available
        for interrupt in self.listOfInterrupts:
            if(ID == interrupt[0]): #check ID
                if(TYPE == "request"):#if request, update
                    if(interrupt[2] > float(PARAMS[0])):#check if the current distance is shorter
                        interrupt[2] = float(PARAMS[0])#update distance
                        interrupt[3] = FROM#change botname
                        
                #if "request" or "ignore"
                interrupt[4] += 1 #increment the amount of responses
                if(interrupt[4] >= interrupt[5] and interrupt[3] != "botname"):#if there have been enough responses
                    #create accept string
                    #message values, 
                    TYPEint, FROMint, TOint, URGENCYint, PRIORITYint, IDint, TASKint, PARAMSint = Tasks.ParseMsg(interrupt[1])
                    msgString = createMsgString(TYPE = "accept", FROM = "server", TO = interrupt[3],
                                                URGENCY = URGENCYint, PRIORITY = PRIORITYint, ID = ID, TASK = TASKint, PARAMS = PARAMSint)
                    newMsg = String()
                    newMsg.data = msgString
                    #remove from list of pending interruption
                    self.listOfInterrupts.remove(interrupt)
                    
                    #one less "active" interrupt
                    self.numActiveInterrupts -= 1
                   
                    #publish new response accepting 
                    self.publisher_.publish(newMsg)
                    self.get_logger().info(msgString)
                    
                    #sends the next interrupt
                    self.sendInterruption()
                    
                else:
                    break
            
            
    #addresses user input, 
    def addressUserInput(self, msg):
        msgFields = msg.data.split(" ")
        
        requestsToHear = self.numOfBots
        #if the interrupt was sent to a specific bot
        if(msgFields[2] != "all"):
            requestsToHear = 1
        
        #adds interruption ID to list, note format: ID, msg, minimumDistance, botname, #of requests heard, #of requests to hear 
        self.listOfInterrupts.append([msgFields[5], msg.data, float(9e7), "botname", 0, requestsToHear])
        self.get_logger().info(msg.data)
        #self.publisher_.publish(msg)
        
        #attempt to send this interruption
        self.sendInterruption()
        
      
    #sends an interrupt 
    def sendInterruption(self):
        if(self.numActiveInterrupts < self.maxActiveInterrupts):
            #send interrupt
            if(len(self.listOfInterrupts) > self.numActiveInterrupts ):#if there is an interrupt to send
                botsMSG = String()             
                botsMSG.data = self.listOfInterrupts[self.numActiveInterrupts][1]
                self.publisher_.publish(botsMSG)
                self.numActiveInterrupts += 1
                
                
                
    #updating bot section
    def robot1Loc(self, poseMsg):
        self.detectCollisions(0, poseMsg)
        
    def robot2Loc(self, poseMsg):
        self.detectCollisions(1, poseMsg)
        
    def robot3Loc(self, poseMsg):
        self.detectCollisions(2, poseMsg)
        
    def robot4Loc(self, poseMsg):
        self.detectCollisions(3, poseMsg)
    
    def robot5Loc(self, poseMsg):
        self.detectCollisions(4, poseMsg)
        
    #this is currently not in use
    def detectCollisions(self, currBotIndex, poseMsg):
        #update the bot to new position
        newPos = poseMsg.pose.position
        newOrient = poseMsg.pose.orientation
        
        tempRoll = math.atan2(2*(newOrient.w * newOrient.x + newOrient.y*newOrient.z), 1 - 2*(newOrient.x**2 + newOrient.y**2))
        tempPitch = math.asin(2*(newOrient.w *newOrient.y - newOrient.z*newOrient.x))
        tempYaw = math.atan2(2*(newOrient.w*newOrient.z + newOrient.x*newOrient.y), 1 - 2*(newOrient.y**2 + newOrient.z**2))
        
        self.botPositions[currBotIndex] = (newPos.x, newPos.y, newPos.z, tempRoll, tempPitch, tempYaw)
        #print(self.botPositions[currBotIndex])
        #print(currBotIndex)
        
        #actions that each bot will take to avoid collision
        actions = ["nothing"] * self.numOfBots
        #options are nothing, (move)left/right, stay, backwards
        #backwards: move to the back right 
        
        #flag to determine if a collision action needs to be sent
        collisionFlag = False
        
        #var angle of the given degree IN RADIANS 
        deg45 = math.pi * .25
        deg90 = math.pi * .5
        deg135 = math.pi * .75
        deg225 = math.pi * 1.25
        deg315 = math.pi * 1.75
        
        
        #find groups of bots that are going to collide
        #test for near collision
        for bot1Index in range(1, self.numOfBots + 1):
            x1, y1, z1, roll1, pitch1, yaw1 = self.botPositions[bot1Index - 1]
            yaw1 = Tasks.normalizeRadians(yaw1)
            
            for bot2Index in range(bot1Index + 1, self.numOfBots + 1):
                #calc distance
                x2, y2, z2, roll2, pitch2, yaw2 = self.botPositions[bot2Index - 1]
                distance = Tasks.findDistance(x1, y1, x2, y2)
                if(distance <= self.collisionDist): #determine what to do to avoid collision
                    self.get_logger().info(f"{bot1Index} yaw1 {yaw1}")
                    self.get_logger().info(f"{bot2Index} yaw2 {yaw2}")
                    yaw2 = Tasks.normalizeRadians(yaw2)
                    self.get_logger().info(f"{bot2Index}  after yaw2 {yaw2}")
                    difAngle = yaw2 - yaw1 
                    self.get_logger().info(f"difAngle before {difAngle}")
                    #normalize difference
                    #difAngle = abs(difAngle)
                    difAngle = Tasks.normalizeRadians(difAngle)
                    #if(difAngle >= math.pi * 2):
                      #  difAngle -= math.pi * 2
                      
                    self.get_logger().info(f"{bot1Index} {bot2Index} {distance}")
                    self.get_logger().info(f"difangle {difAngle}")
                     
                        
                    #case A: bots are  simply nearby one another
                    #if(math.abs(difAngle) <= deg45):
                        #Do nothing, continue as normal
                   
                    #case B: bots have a similar angle and may rear-end one another
                    if(distance <= self.collisionDist and difAngle <= deg135):
                        if(x1 >= x2):
                            if(y1 >= y2):#bot 1 is in the "top left" corner compared to bot 2
                                if( yaw2 > deg135 and yaw2 < deg315):
                                    #case B.1: ←1 ←2 or ↑1 ↑2 or ↖1 ↖2
                                    actions[bot1Index -1] = "nothing"
                                    actions[bot2Index -1] = "wait"
                                else:
                                    #case B.2: →1 →2 or ↓1 ↓2 or ↘1 ↘2
                                    actions[bot1Index -1] = "wait"
                                    actions[bot2Index -1] = "nothing"
                            else:#bot 1 is in the "bottom left" corner compared to bot 2
                                if( yaw2 > deg45 and yaw2 < deg225):
                                    #case B.3: ←1 ←2 or ↓1 ↓2 or ↙1 ↙2
                                    actions[bot1Index -1] = "nothing"
                                    actions[bot2Index -1] = "wait"
                                else:
                                    #case B.4: →1 →2 or ↑1 ↑2 or ↗1 ↗2
                                    actions[bot1Index -1] = "wait"
                                    actions[bot2Index -1] = "nothing"
                        else:
                            if(y1 >= y2):
                                if( yaw2 > deg45 and yaw2 < deg225):
                                    #case B.5: →1 →2 or ↑1 ↑2 or ↗1 ↗2
                                    actions[bot1Index -1] = "nothing"
                                    actions[bot2Index -1] = "wait"
                                else:
                                    #case B.6: ←1 ←2 or ↓1 ↓2 or ↙1 ↙2
                                    actions[bot1Index -1] = "wait"
                                    actions[bot2Index -1] = "nothing"
                            else:
                                if( yaw2 > deg135 and yaw2 < deg315):
                                    #case B.7: →1 →2 or ↓1 ↓2 or ↘1 ↘2
                                    actions[bot1Index -1] = "nothing"
                                    actions[bot2Index -1] = "wait"
                                else:
                                    #case B.8: ←1 ←2 or ↑1 ↑2 or ↖1 ↖2
                                    actions[bot1Index -1] = "wait"
                                    actions[bot2Index -1] = "nothing"
                        collisionFlag = True
                    
                    #case C: bots pointed roughly at one another and are heading for a possible head on collision
                    else:
                        actions[bot1Index -1] = "reverse"
                        actions[bot2Index -1] = "nothing"
                        collisionFlag = True
        
        #if a collision is going to happen, tell the bots what to do
        if(collisionFlag == True):
            #print(actions)
            for botNumber in range(1, self.numOfBots + 1):
                msgString = createMsgString(TYPE = "collision", FROM = "server", TO = f"robot{botNumber}",
                                                        URGENCY = "0", PRIORITY = "0", ID = "0", TASK = "collisionAvoidance", PARAMS = [actions[botNumber - 1]])
                self.get_logger().info(msgString)
                colMsg = String()
                colMsg.data = msgString
                self.publisher_.publish(colMsg)
                time.sleep(1)
                
            self.avoidedCollision = True
        #if a collision has finished being avoided, tell all of the bots to stop
        elif(self.avoidedCollision == True):
            self.avoidedCollision = False
            
            for botNumber in range(1, self.numOfBots + 1):
                msgString = createMsgString(TYPE = "collision", FROM = "server", TO = f"robot{botNumber}",
                                                        URGENCY = "0", PRIORITY = "0", ID = "0", TASK = "collisionAvoidance", PARAMS = ["nothing"])
                self.get_logger().info(msgString)
                colMsg = String()
                colMsg.data = msgString
                self.publisher_.publish(colMsg)
                    


def main(args=None):
    
    numOfBots = 0
    #get system argument for num of bots 
    if(len(sys.argv) < 2):#if there is only one arg then no args were given in cmd
        numOfBots = 1
    else:
        numOfBots= int(sys.argv[1])
    
    
    rclpy.init(args=args)
    NewServerNode = ServerNode(numOfBots = numOfBots)
   
    try:
        rclpy.spin(NewServerNode)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    NewServerNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
