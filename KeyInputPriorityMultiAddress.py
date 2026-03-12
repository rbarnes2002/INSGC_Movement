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
        
        self.validKeyList = ['a', 's', 'd', 'f','g', 'q']
        
        self.fileDesc = sys.stdin.fileno()#gets file descriptor of input
        self.restoreTerminal = termios.tcgetattr(self.fileDesc)#save terminal configs/restore later
        
        self.publishInput()

     
    #user interruption input thread
    def publishInput(self):
        
        #prints to user, also sets terminal to raw mode on its way out
        self.printToUser("Please press one of the listed keys to choose a priority, and then press a numbered key to select which bot to send it to.")
        priority = ""
        priorityPrompt = ("Priority key options:"
             f"\n{self.validKeyList[0]} Priority 1 Wait 3 seconds" 
             f"\n{self.validKeyList[1]} Priority 2 Wait 3 second"
             f"\n{self.validKeyList[2]} Priority 3 Wait 3 second"
             f"\n{self.validKeyList[3]} Priority 4 Wait 3 second"
             f"\n{self.validKeyList[4]} Priority 5 Wait 3 second"
             f"\n{self.validKeyList[5]} to quit")
        
        #displays what bot numbers are available for the user to press
        botPrompt = f"Press key 1 through {self.numOfBots} to select a bot"
        
        
        try:
            while(True):
                self.printToUser(priorityPrompt)
                #choose priority level
                keyInput = str(sys.stdin.read(1))#read one char from the input stream
                
                if(keyInput == self.validKeyList[0]):
                    priority = "1"
                elif(keyInput == self.validKeyList[1]):
                    priority = "2"
                elif(keyInput == self.validKeyList[2]):
                    priority = "3"
                elif(keyInput == self.validKeyList[3]):
                    priority = "4"
                elif(keyInput == self.validKeyList[4]):
                    priority = "5"
                elif(keyInput == self.validKeyList[5]):
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
                msg.data = f"interruption server robot{robotID} 0 {priority} {self.InterruptID} wait 3 2"
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
            except ValueError as e:
                self.printToUser("Please enter a Number!")
                
            #if user input is inside of the range of bots
            if((keyBotInput >= 0) and (keyBotInput <= self.numOfBots)):
                return keyBotInput
            
            #if outside range of bots 
            self.printToUser("Number does not correlate to a robot!\n" + botPrompt)
  
  
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
