import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class UserInput(Node):

    def __init__(self):
        super().__init__('UserInputTerminal')
        
        #publisher section
        self.publisher_ = self.create_publisher(String, 'userTopic', 100)
        self.InterruptID = 0
        
        self.publishInput()

     
    #user interruption input thread
    def publishInput(self):
        while(True):
            interruption = input("Please enter an interruption: ")
            
            #counts the number of spaces found, this is used to know where to insert the interrupt id 
            spaceCounter = 0
            
            #used to increment through interruption
            insertIndex = 0
            
            #this adds the interrupt ID to the interruption, must be done like this since it is unknown what the message
            # will exactly look like/ the number of fields in the message
            while(insertIndex < len(interruption)):
                if(interruption[insertIndex] == " "):
                    spaceCounter += 1
                if(spaceCounter == 4):#if insert location is found
                    break
                insertIndex += 1
            
            #create and publish message
            msg = String()
            msg.data = interruption[:insertIndex] + " " + str(self.InterruptID) + interruption[insertIndex:]
            self.InterruptID += 1
            
            self.publisher_.publish(msg)
            #self.get_logger().info(msg.data)
            
             
         
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
