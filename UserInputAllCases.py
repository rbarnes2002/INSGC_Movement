import rclpy, sys, tty, termios
from rclpy.node import Node

from std_msgs.msg import String
import random
import time 

import argparse


class UserInput(Node):

    def __init__(self, numOfBots, taskOrderType, experimentRunTime, experimentAddressType, taskList, taskListPrompts):
        super().__init__('UserInputTerminal')
        
        #publisher section
        self.publisher_ = self.create_publisher(String, 'userTopic', 100)
        self.InterruptID = 0
        
        self.numOfBots = numOfBots
        self.taskOrderType = taskOrderType
        self.experimentRunTime = experimentRunTime
        self.experimentAddressType = experimentAddressType
        
        #if none given set default values
        if (len(taskList) == 0):
            taskList = ["wait 3 2"]
            if(len(taskListPrompts) == 0):
                taskListPrompts = ["wait 3 seconds"]
        self.taskList = taskList
        self.taskListPrompts = taskListPrompts
        
        self.timeStart = time.time()
        
        #restore terminal stuff
        self.fileDesc = sys.stdin.fileno()#gets file descriptor of input
        self.restoreTerminal = termios.tcgetattr(self.fileDesc)#save terminal configs/restore later
        
        self.publishInput()

     
    #user interruption input thread
    def publishInput(self):
        
        maxTaskIndex = len(self.taskList) -1
        maxTaskPromptIndex = len(self.taskListPrompts) -1
        
        robot = "all"
        priority = 0
        urgency = 0
        taskIndex = 0
        
        #shows the different options that the user has for tasks to send, defaults to the actuall task command itself if a replacement task-prompt has not been given
        taskPrompt = "\nPlease select a task to send! To do so please enter the corresponding number!\n"
        #build task prompt
        if(maxTaskIndex != 0):
            for index in range(0, maxTaskIndex + 1):
                if(index <= maxTaskPromptIndex):
                    taskPrompt += f"{index}: {self.taskListPrompts[index]} \n"
                else:
                    taskPrompt += f"{index}: {self.taskList[index]} \n"
        
        #main loop
        try:
            while(True):
                
                userShownInterrupt = ""
                self.printToUser("Send a new task! Please use the number keys to make your selections!")
               
                #remember, the "group" type of experiments mean that the task is sent to a specific robot, yes this is very confusing
                #if group get bot number
                if(self.experimentAddressType == "group"):
                    self.printToUser(f"\nPlease choose a robot! Hit any key between 1 to {self.numOfBots}")
                    robot = f"robot{self.getNumberFromUser(1, self.numOfBots)}"
                    userShownInterrupt += f"{robot} "
                
                # task orders 
                if(self.taskOrderType == "urgency"):
                    self.printToUser(f"\nPlease choose whether or not the task is urgent! Urgent: 1 Nonurgent: 0")
                    urgency = self.getNumberFromUser(0, 1)
                    userShownInterrupt += f"Urgency: {urgency} "
                elif(self.taskOrderType == "priority"):
                    self.printToUser(f"\nPlease choose a priority for the task! lowest priority is 0 and Highest priority is 5 ")
                    priority = self.getNumberFromUser(0, 5)
                    userShownInterrupt += f"Priority: {priority} "
                else:#for "both" or defaults
                    self.printToUser(f"\nPlease choose whether or not the task is urgent! Urgent: 1 Nonurgent: 0")
                    urgency = self.getNumberFromUser(0, 1)
                    userShownInterrupt += f"Urgency: {urgency} "
                    
                    self.printToUser(f"\nPlease choose a priority for the task! lowest priority is 0 and Highest priority is 5 ")
                    priority = self.getNumberFromUser(0, 5)
                    userShownInterrupt += f"Priority: {priority} "
                    
                
                #if there is more than one task to send, prompt user for which task to send
                if(maxTaskIndex != 0):#check if there is more than one task
                    self.printToUser(taskPrompt)
                    taskIndex = self.getNumberFromUser(0, maxTaskIndex)
                
                if(taskIndex <= maxTaskPromptIndex):
                    userShownInterrupt += self.taskListPrompts[taskIndex]
                else:
                    userShownInterrupt += self.taskList[taskIndex]
                    
                
                #create interruption, print/publish
                interruption = f"interruption server {robot} {urgency} {priority} {self.InterruptID} {self.taskList[taskIndex]}"
                #self.printToUser(interruption)
                msg = String()
                msg.data = interruption
                self.publisher_.publish(msg)
                
                self.printToUser(f"You sent the task: {userShownInterrupt}\n")
                
                self.InterruptID += 1

                if(self.experimentRunTime > 0):
                    if((time.time() - self.timeStart) > self.experimentRunTime):
                        self.printToUser("The Experiment has finished")
                        break

                        
        finally:
            #restore terminal
            termios.tcsetattr(self.fileDesc, termios.TCSADRAIN, self.restoreTerminal)
            
    
    #in order to properly print to the user, the terminal must be restored, and then set back to whitespaceless mode
    def printToUser(self, printStr):
        #restore terminal
        termios.tcsetattr(self.fileDesc, termios.TCSADRAIN, self.restoreTerminal)
        print(printStr)
        #set terminal to raw mode, disables line buffering
        tty.setraw(self.fileDesc)
    
    #gets a number from the user between minInt and maxInt, inclusive
    def getNumberFromUser(self, minInt, maxInt):
        while(True):
            userIntInput = 0
            try:
                userIntInput = int(sys.stdin.read(1))
            except ValueError as e:
                self.printToUser("Please enter a Number!")
                continue
            
            if((userIntInput >= minInt) and (userIntInput <= maxInt)):
                    return userIntInput
            else:
                self.printToUser(f"Please enter a number between {minInt} and {maxInt} inclusive!")
         
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
    argParseObj.add_argument("--add-task",  action="append", default=[], type=str, help="Add a task that can be sent, add a --add-task for each task and args combo. Ex: --add-task \"wait 3 2\" --add-task \"wait 5 2\" \"--add-task moveCardinalDirection west 10\"")
    argParseObj.add_argument("--add-task-prompt", action="append", default=[], type=str, 
                                            help="Add a prompt for each task, this cleans up/changes how the users are shown the task that they sent, for example they can be shown \"Wait 3 seconds\" instead of wait 3 2. \n Note: There should be as many prompts as there are tasks")
    
    args = argParseObj.parse_args()
    
    rclpy.init()
    try:
        userInputNode = UserInput(numOfBots=args.numOfBots, 
                                            taskOrderType=args.task_order_type, 
                                            experimentRunTime=args.experiment_run_time, 
                                            experimentAddressType=args.experiment_address_type, 
                                            taskList=args.add_task,
                                            taskListPrompts=args.add_task_prompt)
    finally:
        userInputNode.destroy_node()
        rclpy.shutdown()
  


if __name__ == '__main__':
    main()
