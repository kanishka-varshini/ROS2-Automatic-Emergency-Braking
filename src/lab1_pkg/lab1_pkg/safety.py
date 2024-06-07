import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety')
        self.subscription1 = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.subscription2 = self.create_subscription(Odometry, 'ego_racecar/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        
        self.latest_odom = None
        self.latest_scan = None
        self.odom_received = False
        self.scan_received = False

    def scan_callback(self,msg):
        self.latest_scan=msg
        self.scan_received= True
        self.try_compute_ittc()
    
    def odom_callback(self,msg):
        self.latest_odom=msg
        self.odom_received= True
        self.try_compute_ittc()

    def try_compute_ittc(self):
        if self.odom_received==True and self.scan_received==True:
            self.compute_ittc()
            self.odom_received = False
            self.scan_received = False
    
    def compute_ittc(self):
        #extract linear velocity in x (forward backward) and the angular velocity abt z
        linear_velocity = self.latest_odom.twist.twist.linear.x
        angular_velocity_z = self.latest_odom.twist.twist.angular.z

        max_angular_velocity=abs(angular_velocity_z)

        ttc_values = []
        angle_increment = self.latest_scan.angle_increment
        for i, distance in enumerate(self.latest_scan.ranges):
            if distance == float('inf'):
                ttc_values.append(float('inf'))
                continue

            #the angle of the current laser beam
            angle = self.latest_scan.angle_min + i * angle_increment

            # effective linear velocity (considering rotation) if no rotation then max_angular_velocity will be zero
            effective_linear_velocity = linear_velocity * math.cos(angle)
            effective_relative_velocity = effective_linear_velocity-max_angular_velocity * distance

            if effective_relative_velocity <= 0: #the obstacle is getting further away
                ttc = float('inf')
            else:
                ttc = distance / effective_relative_velocity
           

            ttc_values.append(ttc)

        # most urgent collision threat
        min_ttc = min(ttc_values)

        if min_ttc<10:
            stop_msg = AckermannDriveStamped()
            stop_msg.drive.speed = 0.0
            stop_msg.drive.steering_angle = 0.0

            self.publisher.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
