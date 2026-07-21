import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import numpy as np
import math
import heapq

# Import TF2 for accurate coordinate frame alignment
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class DijkstraExplorer(Node):
    def __init__(self):
        super().__init__('dijkstra_explorer')
        
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Initialize TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.plan_timer = self.create_timer(2.0, self.replan_loop)
        
        self.map_data = None
        self.map_info = None
        self.goal_pose = None
        self.path = []
        self.is_planning = False

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

    def inflate_obstacles(self, grid):
        inflation_radius_meters = 0.20
        resolution = self.map_info.resolution
        if resolution == 0: 
            return grid
        
        r = int(math.ceil(inflation_radius_meters / resolution))
        inflated_grid = np.copy(grid)
        obs_y, obs_x = np.where(grid == 1)
        
        for y, x in zip(obs_y, obs_x):
            y_min = max(0, y - r)
            y_max = min(self.map_info.height, y + r + 1)
            x_min = max(0, x - r)
            x_max = min(self.map_info.width, x + r + 1)
            inflated_grid[y_min:y_max, x_min:x_max] = 1
            
        return inflated_grid

    def map_callback(self, msg):
        self.map_info = msg.info
        grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        base_map = np.where(grid > 50, 1, 0)
        self.map_data = self.inflate_obstacles(base_map)

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.get_logger().info("Goal received! Initiating planning...")
        self.replan_loop()

    def world_to_grid(self, x, y):
        gx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return (gx, gy)

    def grid_to_world(self, gx, gy):
        x = gx * self.map_info.resolution + self.map_info.origin.position.x
        y = gy * self.map_info.resolution + self.map_info.origin.position.y
        return (x, y)

    def replan_loop(self):
        if self.map_data is None or self.goal_pose is None or self.is_planning:
            return
            
        self.is_planning = True
        self.calculate_dijkstra_path()
        self.is_planning = False

    def calculate_dijkstra_path(self):
        curr_x, curr_y, _ = self.get_robot_pose()
        if curr_x is None:
            self.get_logger().warn("Waiting for TF map->base_footprint transform...")
            return

        start = self.world_to_grid(curr_x, curr_y)
        goal = self.world_to_grid(self.goal_pose.position.x, self.goal_pose.position.y)

        if not (0 <= goal[0] < self.map_info.width and 0 <= goal[1] < self.map_info.height):
            self.get_logger().warn("Goal is outside the current map bounds!")
            return

        queue = [(0, start)]
        distances = {start: 0}
        came_from = {start: None}
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]

        goal_reached = False
        while queue:
            current_dist, current_node = heapq.heappop(queue)

            if current_node == goal:
                goal_reached = True
                break

            for dx, dy in directions:
                neighbor = (current_node[0] + dx, current_node[1] + dy)

                if 0 <= neighbor[0] < self.map_info.width and 0 <= neighbor[1] < self.map_info.height:
                    if self.map_data[neighbor[1], neighbor[0]] == 1:
                        continue 
                    
                    cost = math.sqrt(dx**2 + dy**2)
                    new_dist = current_dist + cost
                    
                    if neighbor not in distances or new_dist < distances[neighbor]:
                        distances[neighbor] = new_dist
                        heapq.heappush(queue, (new_dist, neighbor))
                        came_from[neighbor] = current_node

        if goal_reached:
            new_path = []
            curr = goal
            while curr in came_from:
                new_path.append(self.grid_to_world(curr[0], curr[1]))
                curr = came_from[curr]
            new_path.reverse()
            self.path = new_path
            self.get_logger().info("Path generated successfully.")
        else:
            self.get_logger().warn("No viable path to goal found with current map.")

    def control_loop(self):
        if not self.path:
            return

        curr_x, curr_y, yaw = self.get_robot_pose()
        if curr_x is None:
            return

        target_x, target_y = self.path[0]
        distance_to_target = math.sqrt((target_x - curr_x)**2 + (target_y - curr_y)**2)
        
        if distance_to_target < 0.2:
            self.path.pop(0)
            if not self.path:
                self.cmd_pub.publish(Twist())
                self.goal_pose = None
                self.get_logger().info("Goal Reached!")
            return

        angle_to_target = math.atan2(target_y - curr_y, target_x - curr_x)
        angle_diff = math.atan2(math.sin(angle_to_target - yaw), math.cos(angle_to_target - yaw))

        msg = Twist()
        if abs(angle_diff) > 0.3:
            msg.angular.z = 0.5 if angle_diff > 0 else -0.5
        else:
            msg.linear.x = 0.15
            msg.angular.z = 0.5 * angle_diff
            
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DijkstraExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
