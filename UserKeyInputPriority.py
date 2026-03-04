import rclpy, sys, tty, termios
from rclpy.node import Node

from std_msgs.msg import String



class UserInput(Node):

    def __init__(self):
        super().__init__('UserInputTerminal')
        
        #publisher section
        self.publisher_ = self.create_publisher(String, 'userTopic', 100)
        self.InterruptID = 0
        
        self.validKeyList = ['a', 's', 'd', 'f','g', 'q']
        
        self.publishInput()

     
    #user interruption input thread
    def publishInput(self):
    
        fileDesc = sys.stdin.fileno()#gets file descriptor of input
        restoreTerminal = termios.tcgetattr(fileDesc)#save terminal configs/restore later
        
        interruption = ""
        userPrompt = ("User key options:"
             f"\n{self.validKeyList[0]} Priority 1 Wait 10 seconds" 
             f"\n{self.validKeyList[1]} Priority 2 Wait 10 second"
             f"\n{self.validKeyList[2]} Priority 3 Wait 10 second"
             f"\n{self.validKeyList[3]} Priority 4 Wait 10 second"
             f"\n{self.validKeyList[4]} Priority 5 Wait 10 second"
             f"\n{self.validKeyList[5]} to quit")
             
        print(userPrompt)
        try:
            tty.setraw(fileDesc)#set terminal to raw mode, disables line buffering
            while(True):
                keyInput = str(sys.stdin.read(1))#read one char from the input stream
                
                if(keyInput == self.validKeyList[0]):
                    interruption = f"interruption server all 0 1 {self.InterruptID} wait 10 4"
                elif(keyInput == self.validKeyList[1]):
                    interruption = f"interruption server all 0 2 {self.InterruptID} wait 10 4"
                elif(keyInput == self.validKeyList[2]):
                    interruption = f"interruption server all 0 3 {self.InterruptID} wait 10 4"
                elif(keyInput == self.validKeyList[3]):
                    interruption = f"interruption server all 0 4 {self.InterruptID} wait 10 4"
                elif(keyInput == self.validKeyList[4]):
                    interruption = f"interruption server all 0 5 {self.InterruptID} wait 10 4"
                elif(keyInput == self.validKeyList[5]):
                    print("Quitting..")
                    break
                else:
                    print("Please enter valid input")
                    print(userPrompt)
                    continue
            
                #create and publish message
                msg = String()
                msg.data = interruption
                self.InterruptID += 1

                self.publisher_.publish(msg)
                #self.get_logger().info(msg.data)
        finally:
            #restore terminal
            termios.tcsetattr(fileDesc, termios.TCSADRAIN, restoreTerminal)
            
             
         
def main(args=None):
    rclpy.init(args=args)

    userInputNode = UserInput()

    rclpy.spin(userInputNode)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    userInputNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
