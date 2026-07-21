import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import math

# Import TF2 for accurate coordinate frame alignment
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class APFController(Node):
    def __init__(self):
        super().__init__('apf_controller')
        
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Initialize TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.goal_pose = None
        self.laser_ranges = []
        self.laser_angle_min = 0.0
        self.laser_angle_increment = 0.0
        
        self.zeta = 0.8
        self.eta = 0.05
        self.rho_0 = 0.4

    # Function to get the robot's pose in the 'map' frame using TF2
    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            # Convert quaternion to yaw
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return x, y, yaw
        except Exception as e:
            return None, None, None

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.get_logger().info("New APF Goal Received!")

    def scan_callback(self, msg):
        self.laser_ranges = msg.ranges
        self.laser_angle_min = msg.angle_min
        self.laser_angle_increment = msg.angle_increment

    def control_loop(self):
        if self.goal_pose is None:
            return

        curr_x, curr_y, yaw = self.get_robot_pose()
        if curr_x is None:
            self.get_logger().warn("Waiting for TF map->base_footprint transform...")
            return

        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y

        dist_to_goal = math.sqrt((goal_x - curr_x)**2 + (goal_y - curr_y)**2)
        if dist_to_goal < 0.15:
            self.cmd_pub.publish(Twist())
            self.goal_pose = None
            self.get_logger().info("Goal Reached!")
            return

        angle_to_goal = math.atan2(goal_y - curr_y, goal_x - curr_x)
        f_att_x = self.zeta * dist_to_goal * math.cos(angle_to_goal)
        f_att_y = self.zeta * dist_to_goal * math.sin(angle_to_goal)

        f_rep_x = 0.0
        f_rep_y = 0.0
        
        if self.laser_ranges:
            for i, distance in enumerate(self.laser_ranges):
                if math.isinf(distance) or math.isnan(distance):
                    continue
                
                if distance < self.rho_0 and distance > 0.12:
                    rep_magnitude = self.eta * (1.0/distance - 1.0/self.rho_0) * (1.0/(distance**2))
                    obs_angle = yaw + self.laser_angle_min + (i * self.laser_angle_increment)
                    
                    # 1. Standard Repulsive Force (Pushes strictly away)
                    f_rep_x -= rep_magnitude * math.cos(obs_angle)
                    f_rep_y -= rep_magnitude * math.sin(obs_angle)

                    # 2. Tangential "Swirl" Force (Pushes around the obstacle)
                    angle_diff = math.atan2(math.sin(obs_angle - angle_to_goal), math.cos(obs_angle - angle_to_goal))
                    direction = 1 if angle_diff > 0 else -1
                    tangent_angle = obs_angle + (direction * math.pi/2)
                    
                    f_rep_x -= (rep_magnitude * 0.8) * math.cos(tangent_angle)
                    f_rep_y -= (rep_magnitude * 0.8) * math.sin(tangent_angle)

        f_total_x = f_att_x + f_rep_x
        f_total_y = f_att_y + f_rep_y

        force_angle = math.atan2(f_total_y, f_total_x)
        force_magnitude = math.sqrt(f_total_x**2 + f_total_y**2)

        angle_diff = force_angle - yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        msg = Twist()
        if abs(angle_diff) > 0.4:
            msg.angular.z = 0.5 if angle_diff > 0 else -0.5
            msg.linear.x = 0.0
        else:
            msg.linear.x = min(0.15, force_magnitude)
            msg.angular.z = 0.8 * angle_diff
            
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = APFController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
