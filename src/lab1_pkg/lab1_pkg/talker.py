import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class talker(Node):
    def __init__(self):
        super().__init__("talker")
        
        #publish the message of type AckermannDriveStamped to a topic named drive
        self.pub_= self.create_publisher(AckermannDriveStamped, "drive", 10) 
        self.v = self.declare_parameter('v', 0.0).value
        self.d = self.declare_parameter('d', 0.0).value

        self.timer = self.create_timer(0.01, self.publish_message)
        
        
        self.msg=AckermannDriveStamped()


    def publish_message(self):
        self.msg.drive.speed = self.v
        self.msg.drive.steering_angle = self.d
        self.pub_.publish(self.msg)

def main():
    rclpy.init()
    node=talker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()