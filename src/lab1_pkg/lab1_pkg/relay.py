import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class RelayNode(Node):
    def __init__(self):
        super().__init__('relay')
        self.subscription = self.create_subscription(AckermannDriveStamped, 'drive', self.callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)

    def callback(self, msg):
        speed = msg.drive.speed 
        steering_angle = msg.drive.steering_angle 

        new_msg = AckermannDriveStamped()
        new_msg.drive.speed = speed
        new_msg.drive.steering_angle = steering_angle

        self.publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RelayNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
