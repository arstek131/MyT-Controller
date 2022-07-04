import rclpy
from rclpy.node import Node

from math import *
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from transforms3d import euler
from sensor_msgs.msg import Range


class ThymioController(Node):

    MAX_RANGE = 0.12
    TARGET_DIST = MAX_RANGE - 0.01
    TARGET_ERROR = 0.002

    def __init__(self):
        super().__init__('thymio_controller_2')

        self.name = 'thymio0'
        self.pose = Pose2D()
        self.vel_msg = Twist()
        self.right = None
        self.left = None
        self.linear_speed_x = 2.0
        self.angular_speed_z = 0.0
        self.TURNED = False
        self.FINISHED = False
        self.WALL = False

        self.vel_publisher = self.create_publisher(Twist, f'/{self.name}/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Odometry, f'/{self.name}/odom', self.pose_update_callback, 10)
        self.sensor_left = self.create_subscription(Range, f'/{self.name}/proximity/center_left', self.sensor_callback_left, 10)
        self.sensor_right = self.create_subscription(Range, f'/{self.name}/proximity/center_right', self.sensor_callback_right, 10)


    def sensor_callback_left(self, data):
        self.left = data.range

    def sensor_callback_right(self, data):
        self.right = data.range

    def update_callback(self):
       cmd_vel = Twist()
       cmd_vel.linear.x = self.linear_speed_x
       cmd_vel.angular.z = self.angular_speed_z
       self.vel_publisher.publish(cmd_vel)

       if(self.left == None or self.right == None):
            self.get_logger().info("No wall sensed yet")
            return
       if((min(abs(self.left), abs(self.right)) > self.TARGET_DIST) and not self.WALL and (self.left != -1.0 or self.right != -1.0)):
            self.linear_speed_x = 0.0
            self.get_logger().info("Wall detected!")
            self.WALL = True
       elif(self.WALL and not self.TURNED):
            self.turns_in_place()
       if(self.TURNED and self.FINISHED):
            self.get_logger().info("Done")
            return
           
    def turns_in_place(self):
        angular_value = abs(self.left-self.right)/10.0
        error = abs(self.left-self.right)
        if(error > self.TARGET_ERROR):
            if(self.left > self.right):
                self.angular_speed_z = angular_value
                self.get_logger().info("Turning left")
            else:
                self.angular_speed_z = -angular_value
                self.get_logger().info("Turning right")
        else:
            self.angular_speed_z = 0.0
            self.TURNED = True
            self.FINISHED = True
        return

    def run(self):
        self.timer = self.create_timer(0.1, self.update_callback)
        
    def stop_moving(self):
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
        return

    def pose_update_callback(self, data):
        self.vel_msg = data.twist.twist
        self.pose = self.pose_convers(data.pose.pose)
    
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


def main():
    
    rclpy.init()
    node = ThymioController()
    done = node.run()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.stop_moving()
    

if __name__ == '__main__':
    main()

