import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from Tasks import createMsgString

import threading
import Tasks


class ServerNode(Node):

    def __init__(self):
        super().__init__('server_1')
        
        #publisher section
        self.publisher_ = self.create_publisher(String, '/ServerPublish', 100) 
        
        #timer_period = 5.0  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.InterruptID = 0
        #self.i = 0
        
        #number of bots in the simulation
        self.numOfBots = 2
        
        #subsriber section, listens to the bots
        self.subscription_1 = self.create_subscription(String, '/BotPublish', self.serverListener, 100)
        
        #listens to the user
        self.subscription_2 = self.create_subscription(String, 'userTopic', self.addressUserInput, 100)
        
        self.listOfInterrupts = []
        
        
    #listens and publishes, responds to bot requests
    def serverListener(self, msg):
        #parse msg string, note when hearing a request the params are [distance]
        TYPE, FROM, TO, URGENCY, ID, TASK, PARAMS = Tasks.ParseMsg(msg.data)
        
        #check if interruption is still available
        for interrupt in self.listOfInterrupts:
            if(ID == interrupt[0]): #check ID
                if(TYPE == "request"):#if request, update
                    if(interrupt[2] > float(PARAMS[0])):#check if the current distance is shorter
                        interrupt[2] = float(PARAMS[0])#update distance
                        interrupt[3] = FROM#change botname
                
                #if "request" or "ignore"
                interrupt[4] += 1 #increment the amount of responses
                if(interrupt[4] >= self.numOfBots):#if there have been enough responses
                    #create accept string
                    #message values, 
                    TYPEint, FROMint, TOint, URGENCYint, IDint, TASKint, PARAMSint = Tasks.ParseMsg(interrupt[1])
                    msgString = createMsgString(TYPE = "accept", FROM = "server", TO = interrupt[3],
                                                URGENCY = URGENCYint, ID = ID, TASK = TASKint, PARAMS = PARAMSint)
                    newMsg = String()
                    newMsg.data = msgString
                    #remove from list of pending interruption
                    self.listOfInterrupts.remove(interrupt)
                   
                    #publish new response accepting 
                    self.publisher_.publish(newMsg)
                    self.get_logger().info(msgString)
            
     
    #addresses user input, 
    def addressUserInput(self, msg):
        msgFields = msg.data.split(" ")
        #adds interruption ID to list, note format: ID, msg, minimumDistance, botname, #of requests to hear
        self.listOfInterrupts.append([msgFields[4], msg.data, float(9e7), "botname", 0])
        self.get_logger().info(msg.data)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    NewServerNode = ServerNode()
    rclpy.spin(NewServerNode)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    NewServerNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
