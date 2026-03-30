import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from Tasks import createMsgString

import threading
import Tasks
import sys


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
        
        #self.checkInterruptTimer = self.create_timer(.5, self.sendInterruption)
        
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

def main(args=None):
    
    numOfBots = 0
    #get system argument for num of bots 
    if(len(sys.argv) < 2):#if there is only one arg then no args were given in cmd
        numOfBots = 1
    else:
        numOfBots= int(sys.argv[1])
    
    
    rclpy.init(args=args)
    NewServerNode = ServerNode(numOfBots = numOfBots)
    rclpy.spin(NewServerNode)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    NewServerNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
