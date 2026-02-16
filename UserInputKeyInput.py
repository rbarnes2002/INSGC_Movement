import rclpy
from rclpy.node import Node

from std_msgs.msg import String


from pynput.keyboard import Key, Listener

class UserInput(Node):

    def __init__(self):
        super().__init__('UserInputTerminal')
        
        #publisher section
        self.publisher_ = self.create_publisher(String, 'userTopic', 100)
        self.InterruptID = 0
        
        self.validKeyList = ['a', 's', 'd', 'f', 'e']
        
        print("User key options:" + 
                f"\n{self.validKeyList[0]} NonUrgent Move To (1,1)"+ 
                f"\n{self.validKeyList[1]} NonUrgent Move To (1,-1)"+
                f"\n{self.validKeyList[2]} Urgent Move To (8,8)"+
                f"\n{self.validKeyList[3]} Urgent Move To (8,-8)"+
                f"\n{self.validKeyList[4]} to end user key input.")
        
        print("1")
        self.KeyListener = Listener(on_press = self.onKeyPress)
        print("2")
        self.KeyListener.start()
        print("3")

     
    #user interruption input thread
    def publishInput(self, keyInput):
            interruption = ""
            if(keyInput == validKeyList[0]):
                interruption = f"interruption server all 0 {self.InterruptID} moveto 1 1 1"
            elif(keyInput == validKeyList[1]):
                interruption = f"interruption server all 0 {self.InterruptID} moveto 1 -1 1"
            elif(keyInput == validKeyList[2]):
                interruption = f"interruption server all 5 {self.InterruptID} moveto 8 8 1"
            elif(keyInput == validKeyList[3]):
                interruption = f"interruption server all 5 {self.InterruptID} moveto 8 -8 1"
            elif(keyInput == validKeyList[4]):
                self.KeyListener.stop()
            
            if(interruption != ""):
                #create and publish message
                msg = String()
                msg.data = interruption
                self.InterruptID += 1
                
                self.publisher_.publish(msg)
                #self.get_logger().info(msg.data)
    
    #listens to key inputs, calls to start publishing if valid input
    def onKeyPress(self, key):
        try:
            print("KEY PRESSED")
            keyInput = key.char
            print(keyInput)
            if keyInput in validKeyList:
                self.PublishInput(keyInput)
        except AttributeError:
            keyInput = ""#only here to stop warning
            
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
                
