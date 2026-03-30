import rclpy, sys, tty, termios
import argparse
from rclpy.node import Node

from std_msgs.msg import String



class UserInput(Node):

    def __init__(self, numOfBots):
        super().__init__('UserInputTerminal')
        
        #publisher section
        self.publisher_ = self.create_publisher(String, 'userTopic', 100)
        self.InterruptID = 0
        
        self.numOfBots = numOfBots
        
        self.validKeyList = ['a', 's', 'q']
        
        self.fileDesc = sys.stdin.fileno()#gets file descriptor of input
        self.restoreTerminal = termios.tcgetattr(self.fileDesc)#save terminal configs/restore later
        
        self.publishInput()

     
    #user interruption input thread
    def publishInput(self):
        
        #prints to user, also sets terminal to raw mode on its way out
        self.printToUser("Please press one of the listed keys to choose either urgent or nonurgent, and then press a numbered key to select which bot to send it to.")
        urgency = ""
        urgencyPrompt = ("Urgent key options:"
             f"\n{self.validKeyList[0]} NonUrgent Wait 3 seconds" 
             f"\n{self.validKeyList[1]} Urgent Wait 3 second"
             f"\n{self.validKeyList[2]} to quit")
        
        #displays what bot numbers are available for the user to press
        botPrompt = f"Press key 1 through {self.numOfBots} to select a bot"
        
        try:
            while(True):
                self.printToUser(urgencyPrompt)
                #choose priority level
                keyInput = str(sys.stdin.read(1))#read one char from the input stream
                
                if(keyInput == self.validKeyList[0]):
                    urgency = "0"
                elif(keyInput == self.validKeyList[1]):
                    urgency = "1"
                elif(keyInput == self.validKeyList[2]):
                    self.printToUser("Quitting..")
                    break
                else:
                    #re-prompt user 
                    self.printToUser("Please enter valid input\n")
                    continue
                
                #prompt user to store robot
                robotID = self.promptForBot(botPrompt = botPrompt)
            
                #create and publish message
                msg = String()
                msg.data = f"interruption server robot{robotID} {urgency} 0 {self.InterruptID} wait 3 2"
                self.InterruptID += 1

                self.publisher_.publish(msg)
                #self.get_logger().info(msg.data)
                self.printToUser("Valid Input!\n")
        finally:
            #restore terminal
            termios.tcsetattr(self.fileDesc, termios.TCSADRAIN, self.restoreTerminal)
    
    
    #prompts user for the bot number, must be given a prompt 
    def promptForBot(self, botPrompt):
        
        self.printToUser(botPrompt)
        while(True):
            #check if it is an int 
            try:
                keyBotInput = int(sys.stdin.read(1))
                
                #if user input is inside of the range of bots
                if((keyBotInput >= 0) and (keyBotInput <= self.numOfBots)):
                    return keyBotInput
            
                #if outside range of bots
                self.printToUser("Number does not correlate to a robot!\n" + botPrompt)
            except ValueError as e:
                self.printToUser("Please enter a Number!")

  
  
    #in order to properly print to the user, the terminal must be restored, and then set back to whitespaceless mode
    def printToUser(self, printStr):
        #restore terminal
        termios.tcsetattr(self.fileDesc, termios.TCSADRAIN, self.restoreTerminal)
        print(printStr)
        #set terminal to raw mode, disables line buffering
        tty.setraw(self.fileDesc)
         
def main():
    
    #used to parse the args 
    argParseObj = argparse.ArgumentParser()
    argParseObj.add_argument(
        "--numOfBots",
        type = int,
        default=4,
        help="Number of bots that the user can send interruptions to",
    )
    
    args = argParseObj.parse_args()
    
    rclpy.init()
    userInputNode = UserInput(numOfBots = args.numOfBots)
    
    try:
        rclpy.spin(userInputNode)
    except KeyboardInterrupt:
        pass
    finally:
        #userInputNode.close()
        userInputNode.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
