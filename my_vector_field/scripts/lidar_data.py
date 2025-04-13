import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import numpy as np
import csv

class LaserScanAnalysis(Node):

    def __init__(self):
        super().__init__('laser_scan_analysis_node')
        self.rear_received = False
        self.front_received = False
        self.front_laser_points = np.array([0, 0], dtype=np.float64)
        self.rear_laser_points = np.array([0, 0], dtype=np.float64)
        

        self.front_angle_i = 0.0
        self.front_xi = 0.0
        self.front_yi = 0.0

        self.rear_angle_i = 0.0
        self.rear_xi = 0.0
        self.rear_yi = 0.0

        self.front_laser_subscription_ = self.create_subscription(LaserScan, '/robot/front_laser/scan', self.front_subscription_callback, 10)
        self.rear_laser_subscription_ = self.create_subscription(LaserScan, '/robot/rear_laser/scan', self.rear_subscription_callback, 10)
    
    def front_subscription_callback(self, msg : LaserScan):
        print('Start to save front laser data...')
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min

        for i in range(len(msg.ranges)):
            self.front_angle_i = angle_min + i*angle_increment
            self.front_xi = msg.ranges[i] * np.cos(self.front_angle_i)
            self.front_yi = msg.ranges[i] * np.sin(self.front_angle_i)

            self.front_laser_points = np.append(self.front_laser_points, [self.front_xi, self.front_yi])

        
        self.front_laser_points = self.front_laser_points.reshape(-1, 2)
        self.front_laser_points = np.delete(self.front_laser_points, 0, axis=0)
        # print(msg.range_min)
        # print(msg.range_max)
        # print(self.front_laser_points)
        with open("front_laser_points.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerows(self.front_laser_points)
        print('Save Done')

        self.front_received = True
    
    def rear_subscription_callback(self, msg : LaserScan):
        print('Started to save rear data...')
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min

        for i in range(len(msg.ranges)):
            self.rear_angle_i = angle_min + i*angle_increment
            self.rear_xi = msg.ranges[i] * np.cos(self.rear_angle_i)
            self.rear_yi = msg.ranges[i] * np.sin(self.rear_angle_i)

            self.rear_laser_points = np.append(self.rear_laser_points, [self.rear_xi, self.rear_yi])


        self.rear_laser_points = self.rear_laser_points.reshape(-1, 2)
        self.rear_laser_points = np.delete(self.rear_laser_points, 0, axis=0)


        with open("rear_laser_points.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerows(self.rear_laser_points)
        print('Save Done')
        self.rear_received = True



def main(args=None):
    rclpy.init(args=args)
    laser_scan_analysis = LaserScanAnalysis()

    while rclpy.ok():
        rclpy.spin_once(laser_scan_analysis, timeout_sec=0.1)  # Small timeout is fine
        if laser_scan_analysis.front_received and laser_scan_analysis.rear_received:
            break

if __name__ == '__main__':
    main()


