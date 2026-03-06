import rclpy, sys, tty, termios
from rclpy.node import Node

from std_msgs.msg import String



class UserInput(Node):

    def __init__(self):
        super().__init__('UserInputTerminal')
        
        #publisher section
        self.publisher_ = self.create_publisher(String, 'userTopic', 100)
        self.InterruptID = 0
        
        self.validKeyList = ['a', 's','q']
        self.urgency_interrupt = False; 
        self.publishInput()

     
    #user interruption input thread
    def publishInput(self):
    
        fileDesc = sys.stdin.fileno()#gets file descriptor of input
        restoreTerminal = termios.tcgetattr(fileDesc)#save terminal configs/restore later
        
        interruption = ""
        userPrompt = ("User key options:"
             f"\n{self.validKeyList[0]} NonUrgent Move To (-70, -63)" 
             f"\n{self.validKeyList[1]} Urgent Move To (-40, -60)"
             f"\n{self.validKeyList[2]} to quit")
             
        print(userPrompt)
        try:
            tty.setraw(fileDesc)#set terminal to raw mode, disables line buffering
            while(True):
                keyInput = str(sys.stdin.read(1))#read one char from the input stream
                
                if(keyInput == self.validKeyList[0]):
                    self.urgency_interrupt = False; 
                    interruption = f"interruption server all 0 2 {self.InterruptID} moveto -70 -63 .6"
                elif(keyInput == self.validKeyList[1]):
                    self.urgency_interrupt = True; 
                    interruption = f"interruption server all 1 2 {self.InterruptID} moveto -40 -60 .6"
                elif(keyInput == self.validKeyList[2]):
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
