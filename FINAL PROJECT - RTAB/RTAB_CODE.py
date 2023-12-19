#Kalaipriyan R - kr53

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time

class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        self.subscription_rtabmap = self.create_subscription(Odometry,'/rtabmap/odom',self.rtabmap_callback,10)
        self.subscription_ekf = self.create_subscription(Odometry, '/terrasentia/ekf', self.ekf_callback, 10)
        self.rtabmap_data = []
        self.ekf_data = []

    def ekf_callback(self, msg):
        self.get_logger().info('Received EKF data')
        position = msg.pose.pose.position 
        self.ekf_data.append([position.x, position.y, position.z])
        print("EKF data appended, current length:", len(self.ekf_data))

    def rtabmap_callback(self, msg):
        self.get_logger().info('Received RTAB-Map data')
        position = msg.pose.pose.position  
        self.rtabmap_data.append([position.x, position.y, position.z])
        print("RTAB-Map data appended, current length:", len(self.rtabmap_data))

    def save_data_to_file(self, filename, data):
        np.savetxt(filename, np.array(data), delimiter=',')
        print("Data shape:", np.array(data).shape)

    def plot_3d_trajectory(self, data, title, file_name):
        data = np.array(data)
        if data.ndim == 2 and data.shape[1] == 3:  
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.plot(data[:, 0], data[:, 1], data[:, 2], label=title)
            ax.set_xlabel('X-axis')
            ax.set_ylabel('Y-axis')
            ax.set_zlabel('Z-axis')
            ax.legend()
            plt.title(title)
            plt.savefig(file_name)
            plt.close(fig)
        else:
            print("Data is not in the correct format for plotting.")

    def compute_rmse(self, estimated_data, ground_truth_data):
        if len(estimated_data) == len(ground_truth_data):
            rmse = np.sqrt(np.mean((np.array(estimated_data) - np.array(ground_truth_data))**2, axis=0))
            return rmse
        else:
            print("Data arrays have mismatched lengths, cannot compute RMSE")
            return None

    def save_and_plot_trajectories(self):
        if len(self.rtabmap_data) > 0 and len(self.ekf_data) > 0:
            rtabmap_data_np = np.array(self.rtabmap_data)
            ekf_data_np = np.array(self.ekf_data)
            
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            
            if rtabmap_data_np.ndim == 2 and rtabmap_data_np.shape[1] == 3:
                ax.plot(rtabmap_data_np[:, 0], rtabmap_data_np[:, 1], rtabmap_data_np[:, 2], label='RTAB-Map Trajectory')
            else:
                print("RTAB-Map data is not in the correct format for plotting.")
            
            if ekf_data_np.ndim == 2 and ekf_data_np.shape[1] == 3:
                ax.plot(ekf_data_np[:, 0], ekf_data_np[:, 1], ekf_data_np[:, 2], label='EKF Trajectory')
            else:
                print("EKF data is not in the correct format for plotting.")
            
            ax.set_xlabel('X-axis')
            ax.set_ylabel('Y-axis')
            ax.set_zlabel('Z-axis')
            ax.legend()
            plt.title('Trajectories Comparison')
            
            # Save the combined figure
            plt.savefig('combined_trajectory.png')
            plt.close(fig)

            self.save_data_to_file('rtabmap_trajectory.txt', self.rtabmap_data)
            self.save_data_to_file('ekf_trajectory.txt', self.ekf_data)

            # Computing RMSE
            rmse = self.compute_rmse(self.rtabmap_data, self.ekf_data)
            print(f'RMSE: {rmse}')
        else:
            print("Insufficient data to plot trajectories.")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()

    try:
        rclpy.spin(node)  
    except KeyboardInterrupt:
        pass
    finally:
        node.save_and_plot_trajectories()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
