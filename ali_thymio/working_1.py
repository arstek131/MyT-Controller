import cmd
import rclpy
from rclpy.node import Node
from rclpy.task import Future

from math import *
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from transforms3d import euler
import time

class ThymioController(Node):
    def __init__(self):
        super().__init__('thymio_controller')
    
        self.name = 'thymio0'

        self.linear_speed = 10.0
        self.radius = 0.66
        self.angular_speed = self.linear_speed / self.radius
        self.circumference = 2 * pi * self.radius

        self.time_needed = self.circumference / self.linear_speed

        self.vel_publisher = self.create_publisher(Twist, f'/{self.name}/cmd_vel', 10)

        self.pose_subscriber = self.create_subscription(Odometry, f'/{self.name}/odom', self.pose_update_callback, 10)

        self.time_step = 0
        self.time_clock = 1.0/60
        self.timer_python_start = None



    def start(self):
        self.timer = self.create_timer(1./60, self.update_callback)
            

    def update_callback(self):
        if(self.time_step ==0):
            self.timer_python_start = time.time()

        cmd_vel = Twist()
        cmd_vel.linear.x = self.linear_speed
        cmd_vel.angular.z = self.angular_speed
        self.time_step+=self.time_clock

        if((self.time_step*self.time_clock) < self.time_needed*(4.0/3.0)):
            self.vel_publisher.publish(cmd_vel)
        else:
            self.angular_speed = -self.angular_speed
            self.time_step=0.

    
    def pose_convers(self, pose):

        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )

        roll,pitch, yaw = euler.quat2euler(quaternion)

        result = Pose2D(x = pose.position.x, y = pose.position.y, theta = yaw)
        
        return result

    def pose_update_callback(self, data):
        self.current_pose = data.pose.pose
        self.velocity = data.twist.twist
    
    def stop_moving(self):
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)

    
   

def main():
    
    rclpy.init()

    node = ThymioController()
    node.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.stop_moving()


if __name__ == '__main__':
    main()

