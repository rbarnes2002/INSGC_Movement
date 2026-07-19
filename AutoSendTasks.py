import rclpy, sys, tty, termios
from rclpy.node import Node

from std_msgs.msg import String
import random
import time 

import argparse


class UserInput(Node):

    def __init__(self, numOfBots, taskOrderType, experimentRunTime, experimentAddressType, maxWaitTime, taskList):
        super().__init__('UserInputTerminal')
        
        #publisher section
        self.publisher_ = self.create_publisher(String, 'userTopic', 100)
        self.InterruptID = 0
        
        self.numOfBots = numOfBots
        self.taskOrderType = taskOrderType
        self.experimentRunTime = experimentRunTime
        self.experimentAddressType = experimentAddressType
        self.maxWaitTime = maxWaitTime
        if (len(taskList) == 0):
            taskList = ["wait 3 2"]
        self.taskList = taskList
        
        self.timeStart = time.time()
        
        self.publishInput()

     
    #user interruption input thread
    def publishInput(self):
        
        maxTaskIndex = len(self.taskList) -1
        
        #main loop
        while(True):
            robot = "all"
            priority = 0
            urgency = 0
            taskIndex = random.randint(0, maxTaskIndex)
            
            if(self.experimentAddressType == "single"):
                robot = f"robot{random.randint(1, self.numOfBots)}"
            
            if(self.taskOrderType == "urgency"):
                urgency = random.randint(0, 1)
            elif(self.taskOrderType == "priority"):
                priority = random.randint(1, 5)
            else:#for "both" or defaults
                urgency = random.randint(0, 1)
                priority = random.randint(1, 5)

            interruption = f"interruption server {robot} {urgency} {priority} {self.InterruptID} {self.taskList[taskIndex]}"
            self.get_logger().info(interruption)
                
            
            msg = String()
            msg.data = interruption
            self.InterruptID += 1

            self.publisher_.publish(msg)
            
            if(self.experimentRunTime > 0):
                if((time.time() - self.timeStart) > self.experimentRunTime):
                    return
            
            time.sleep(random.randint(0, self.maxWaitTime))
            
             
         
def main():
    
    
    argParseObj = argparse.ArgumentParser()
    argParseObj.add_argument(
        "--numOfBots",
        type = int,
        default=4,
        help="Number of bots that the user can send interruptions to",
    )
    argParseObj.add_argument("--task-order-type", action="store", default="both", help="Determines whether priority, urgency, or both are stored")
    argParseObj.add_argument("--experiment-run-time", action="store", default=300, type=int, help="How long the experiment is, the logger will self-terminate to ensure that data is written to the CSVs. Note: enter 0 to turn off this feature")
    argParseObj.add_argument("--experiment-address-type", action="store", default="group", type=str, help="determines whether the tasks are addressed to specific robots or all of them, options are single or group")
    argParseObj.add_argument("--max-wait-time", action="store", default=20, type=int, help="Maximum time to wait between sending tasks")
    argParseObj.add_argument("--add-task",  action="append", default=[], type=str, help="Add a task that can be sent, add a --add-task for each task and args combo. Ex: --add-task \"wait 3 2\" --add-task \"wait 5 2\" \"--add-task moveCardinalDirection west 10\"")
    
    args = argParseObj.parse_args()
    
    rclpy.init()

    userInputNode = UserInput(numOfBots=args.numOfBots, 
                                            taskOrderType=args.task_order_type, 
                                            experimentRunTime=args.experiment_run_time, 
                                            experimentAddressType=args.experiment_address_type, 
                                            maxWaitTime=args.max_wait_time, 
                                            taskList=args.add_task)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    userInputNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
