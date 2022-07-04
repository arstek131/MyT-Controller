import rclpy
from rclpy.node import Node


from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from transforms3d import euler
from enum import Enum
from math import sqrt
from copy import copy

class ThymioState(Enum):
    STRAIGHT = 1
    TURNED = 2
    BACK = 3

class ThymioController(Node):
    TARGET_WALL = 0.06
    TOLLERANCE = 0.0005
    def __init__(self):
        super().__init__('thymio_controller_3')
        self.name = 'thymio0'
        self.pose = Pose2D()
        self.vel_msg = Twist()

        self.center = -1
        self.center_right = -1
        self.center_left = -1
        self.rear_left = -1
        self.rear_right = -1
        self.left = -1
        self.right = -1
        self.from_back = 0
        self.wall_pose = None

        self.current_state = ThymioState.STRAIGHT

    
        self.vel_publisher = self.create_publisher(Twist, f'/{self.name}/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Odometry, f'/{self.name}/odom', self.pose_update_callback, 10)

        self.sensor_center = self.create_subscription(Range, f'/{self.name}/proximity/center', self.sensor_callback, 10)
        self.sensor_center_left = self.create_subscription(Range, f'/{self.name}/proximity/center_left', self.sensor_callback_center_left, 10)
        self.sensor_center_right = self.create_subscription(Range, f'/{self.name}/proximity/center_right', self.sensor_callback_center_right, 10)
        self.sensor_rear_right = self.create_subscription(Range, f'/{self.name}/proximity/rear_right', self.sensor_callback_rear_right, 10)
        self.sensor_rear_left = self.create_subscription(Range, f'/{self.name}/proximity/rear_left', self.sensor_callback_rear_left, 10)
        self.sensor_left = self.create_subscription(Range, f'/{self.name}/proximity/left', self.sensor_callback_left, 10)
        self.sensor_right = self.create_subscription(Range, f'/{self.name}proximity/right', self.sensor_callback_right, 10)

        
        

        

    def refresh_callback(self):
        # # Wait until we receive the current pose of the thymio for the first time
        if(self.pose is None):
            return
        
        if self.current_state == ThymioState.STRAIGHT:
            # Thymio is going straight
            self.go_straight()
        
        if self.current_state == ThymioState.TURNED:
            # Thymio is turning opposite to the wall
            self.self_turn()

        if self.current_state == ThymioState.BACK:
            # Thymio is moving 2m from the wall
            self.moving_back()


    def moving_back(self):
        if(self.wall_pose is None):
                self.stop_moving()
                self.wall_pose = Pose2D()
                self.wall_pose = copy(self.pose)
        elif(self.euclidean_distance(self.pose, self.wall_pose) + self.from_back < 2): # Computing distance from the wall
                self.velocity_publisher()
                self.get_logger().info("Moving back")
        else:
                self.get_logger().info("Goal reached, 2m")
                self.stop_moving()

        return

    def self_turn(self):
        if((self.rear_right == -1 or self.rear_left == -1) or abs(self.rear_right - self.rear_left) > self.TOLLERANCE):
            self.velocity_publisher(0.0, 0.2, self.direction) # Rotating Thymio
            self.get_logger().info("Rotating...") 
        else:
            self.from_back = (self.rear_right + self.rear_left)/2 # Getting the distance of the wall from rear sensors
            self.stop_moving()
            self.current_state = ThymioState.BACK
        return

    def go_straight(self):
        if(abs(self.center_left) > self.TARGET_WALL and abs(self.center_right) > self.TARGET_WALL and abs(self.left)> self.TARGET_WALL and abs(self.right) > self.TARGET_WALL):
                self.velocity_publisher() # moving thymio straight
                self.get_logger().info("Moving straight")
        else:
            self.get_logger().info("Wall detected")
            self.stop_moving()
            self.direction = abs(self.center_left) >= abs(self.center_right) and 1 or -1
            self.current_state = ThymioState.TURNED
        return
    
    def velocity_publisher(self, linear_x = 0.2, angular_z = 0.0, direction = 1):
        self.vel_msg.linear.x = linear_x
        self.vel_msg.angular.z = angular_z * direction
        self.vel_publisher.publish(self.vel_msg)


    def run(self):
        self.timer = self.create_timer(0.1, self.refresh_callback)

    def stop_moving(self):
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)

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
    
    def sensor_callback_rear_right(self, data):
        self.rear_right = data.range

    def sensor_callback_rear_left(self, data):
        self.rear_left = data.range

    def sensor_callback_center_right(self, data):
        self.center_right = data.range

    def sensor_callback_center_left(self, data):
        self.center_left = data.range

    def sensor_callback_left(self, data):
        self.left = data.range
    
    def sensor_callback_right(self, data):
        self.right = data.range

    def sensor_callback(self, data):
        self.center = data.range

    def euclidean_distance(self, goal_pose, current_pose):
        return sqrt(pow((goal_pose.x - current_pose.x), 2) + pow((goal_pose.y - current_pose.y), 2))


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
