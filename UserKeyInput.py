import rclpy, sys, tty, termios
from rclpy.node import Node

from std_msgs.msg import String



class UserInput(Node):

    def __init__(self):
        super().__init__('UserInputTerminal')
        
        #publisher section
        self.publisher_ = self.create_publisher(String, 'userTopic', 100)
        self.InterruptID = 0
        
        self.validKeyList = ['a', 's', 'd', 'f', 'q', 'g', 'h', 'j', 'k', 'l']
        
        self.publishInput()

     
    #user interruption input thread
    def publishInput(self):
    
        fileDesc = sys.stdin.fileno()#gets file descriptor of input
        restoreTerminal = termios.tcgetattr(fileDesc)#save terminal configs/restore later
        
        interruption = ""
        userPrompt = ("User key options:"
             f"\n{self.validKeyList[0]} NonUrgent Priority 1 Move To (56, 16)" 
             f"\n{self.validKeyList[1]} NonUrgent Priority 1 Move To (56, 14)"
             f"\n{self.validKeyList[2]} Urgent Priority 1 Move To (63, 23)"
             f"\n{self.validKeyList[3]} Urgent Priority 1 Move To (63, 7)"
             f"\n{self.validKeyList[4]} to quit")
             
        print(userPrompt)
        try:
            tty.setraw(fileDesc)#set terminal to raw mode, disables line buffering
            while(True):
                keyInput = str(sys.stdin.read(1))#read one char from the input stream
                
                if(keyInput == self.validKeyList[0]):
                    interruption = f"interruption server all 0 1 {self.InterruptID} moveto 56 16 .6"
                elif(keyInput == self.validKeyList[1]):
                    interruption = f"interruption server all 0 1 {self.InterruptID} moveto 56 14 .6"
                elif(keyInput == self.validKeyList[2]):
                    interruption = f"interruption server all 1 1 {self.InterruptID} moveto 63 23 .6"
                elif(keyInput == self.validKeyList[3]):
                    interruption = f"interruption server all 1 1 {self.InterruptID} moveto 63 7 .6"
                elif(keyInput == self.validKeyList[4]):
                    print("Quitting..")
                    break
                elif(keyInput == self.validKeyList[5]):
                    interruption = f"interruption server all 1 2 {self.InterruptID} moveto 51 11 .6"
                elif(keyInput == self.validKeyList[6]):
                    interruption = f"interruption server all 1 3 {self.InterruptID} moveto 51 9 .6"
                elif(keyInput == self.validKeyList[7]):
                    interruption = f"interruption server all 1 2 {self.InterruptID} moveto 58 18 .6"
                elif(keyInput == self.validKeyList[8]):
                    interruption = f"interruption server all 1 3 {self.InterruptID} moveto 58 2 .6"
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
            termios.tcsetattr(fileDesc, termios.TCSADRAIN, restoreTerminal)
            #restore terminal
             
         
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
