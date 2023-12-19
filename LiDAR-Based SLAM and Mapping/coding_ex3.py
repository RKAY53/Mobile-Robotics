
# Student name: Kalaipriyan R - kr53

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from tf2_msgs.msg import TFMessage
from copy import copy
from visualization_msgs.msg import Marker

# Further info:
# On markers: http://wiki.ros.org/rviz/DisplayTypes/Marker
# Laser Scan message: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html

class CodingExercise3(Node):
    def __init__(self):
        super().__init__('CodingExercise3')

        # Initialize robot position and orientation
        self.robot_position = None
        self.robot_orientation = None

        # Initialize attributes for line marker
        self.point_list = []
        self.line = Marker()
        self.line_marker_init()
        self.global_map = np.array([]).reshape(0, 3) 
        self.grid_resolution = 0.1  
        self.grid_size_x = int(100 / self.grid_resolution)  
        self.grid_size_y = int(100 / self.grid_resolution)  
        self.occupancy_threshold = 3 
        self.grid_origin = np.array([-50, -50])  
        self.occupancy_grid = np.zeros((self.grid_size_x, self.grid_size_y)) 
        
        self.subscription_scan = self.create_subscription(LaserScan, 'terrasentia/scan', self.callback_scan, 10)
        self.subscription_ekf = self.create_subscription(Odometry, '/terrasentia/ekf', self.callback_ekf, 10)
        self.pub_lines = self.create_publisher(Marker, '/lines', 10)
        
    def callback_ekf(self, msg):
        # position
        position = msg.pose.pose.position
        self.robot_position = np.array([position.x, position.y, position.z])

        #orientation as quaternion
        orientation = msg.pose.pose.orientation
        self.robot_orientation = np.array([orientation.x, orientation.y, orientation.z, orientation.w])

    def callback_scan(self, msg):
        if self.robot_position is None or self.robot_orientation is None:
            return

        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        distances = np.array(msg.ranges)
        cartesian_coords = np.array([self.polar_to_cartesian(d, a) for d, a in zip(distances, angles) if d > 0])

        if cartesian_coords.shape[1] != 3:
            cartesian_coords = np.hstack((cartesian_coords, np.zeros((cartesian_coords.shape[0], 1))))

        global_coords = self.transform_to_global_frame(cartesian_coords)
        self.draw_lines(global_coords)
        
    def update_occupancy_grid(self, global_coords):
        grid_indices = np.floor((global_coords - self.grid_origin) / self.grid_resolution).astype(int)
        # Update the occupancy grid
        for index in grid_indices:
            if 0 <= index[0] < self.occupancy_grid.shape[0] and 0 <= index[1] < self.occupancy_grid.shape[1]:
                self.occupancy_grid[index[0], index[1]] += 1  

    def draw_map(self):
        map_marker = Marker()
        map_marker.header.frame_id = "odom"
        map_marker.type = Marker.POINTS
        
        for i in range(self.global_map.shape[0]):
            for j in range(self.global_map.shape[1]):
                if self.global_map[i][j] >= self.occupancy_threshold:
                    p = Point()
                    p.x = (i * self.grid_resolution) + self.grid_origin[0]
                    p.y = (j * self.grid_resolution) + self.grid_origin[1]
                    p.z = 0  
                    map_marker.points.append(p)
        self.pub_lines.publish(map_marker)
        
    def update_global_map(self, new_coords):
        if self.global_map.size == 0:
            self.global_map = new_coords
        else:
            self.global_map = np.vstack((self.global_map, new_coords))

    def polar_to_cartesian(self, rho, theta):
        x = rho * np.cos(theta)
        y = rho * np.sin(theta)
        return np.array([x, y, 0.0])

    def quaternion_to_matrix(self, quaternion):
        x, y, z, w = quaternion
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz, xw, yw, zw = x * y, x * z, y * z, x * w, y * w, z * w

        rotation_matrix = np.array([
            [1 - 2 * (yy + zz),     2 * (xy - zw),     2 * (xz + yw)],
            [    2 * (xy + zw), 1 - 2 * (xx + zz),     2 * (yz - xw)],
            [    2 * (xz - yw),     2 * (yz + xw), 1 - 2 * (xx + yy)]
        ])
        
        return rotation_matrix

    def transform_to_global_frame(self, local_coords):
        rotation_mat = self.quaternion_to_matrix(self.robot_orientation)
        
        if local_coords.shape[1] == 2:
            local_coords_homogeneous = np.hstack((local_coords, np.zeros((local_coords.shape[0], 1))))
        else:
            local_coords_homogeneous = local_coords
        rotated_coords = np.dot(local_coords_homogeneous, rotation_mat.T)
        global_coords = rotated_coords + self.robot_position.reshape(1, 3)  # Ensure robot_position is a (1, 3) matrix

        return global_coords


    def douglas_peucker(self, points, epsilon):
        def perpendicular_distance(point, line_start, line_end):
            if np.all(line_start == line_end):
                return np.linalg.norm(point - line_start)
            return np.abs(np.cross(line_end-line_start, line_start-point)) / np.linalg.norm(line_end-line_start)

        if len(points) < 3:
            return points

        dmax = 0
        index = 0
        for i in range(1, len(points) - 1):
            d = perpendicular_distance(points[i], points[0], points[-1])
            max_d = np.max(d) 
            if max_d > dmax:
                index = i
                dmax = max_d

        if dmax > epsilon:
            rec_results1 = self.douglas_peucker(points[:index+1], epsilon)
            rec_results2 = self.douglas_peucker(points[index:], epsilon)
            return np.vstack((rec_results1[:-1], rec_results2))
        else:
            return np.array([points[0], points[-1]])

    def polar_to_cartesian(self, rho, theta):
        return rho * np.cos(theta), rho * np.sin(theta)

    def draw_lines(self, coordinates):
        self.point_list = []
        self.line.id += 1

        for i in range(0, len(coordinates) - 1, 2):
            
            # Start point of the line segment
            p0 = Point(x=float(coordinates[i][0]), y=float(coordinates[i][1]), z=0.0)
            shorten_factor = 1    #[WHILE SCANNING]
            shorten_factor = 0.01 #[WHILE MAPPING]
            
            #shortened end point
            p1 = Point()
            p1.x = p0.x + (coordinates[i+1][0] - coordinates[i][0]) * shorten_factor
            p1.y = p0.y + (coordinates[i+1][1] - coordinates[i][1]) * shorten_factor
            p1.z = 0.0

            self.point_list.extend([p0, p1])
        self.line.points = self.point_list
        self.pub_lines.publish(self.line)


    def line_marker_init(self):
        self.line.header.frame_id = "/odom"
        self.line.ns = "markers"
        self.line.id = 0
        self.line.type = Marker.LINE_LIST
        self.line.action = Marker.ADD
        self.line.pose.orientation.w = 1.0
        self.line.scale.x = 0.05
        self.line.color.r = 1.0
        self.line.color.a = 1.0

def main(args=None):
    rclpy.init(args=args)
    node = CodingExercise3()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
