import rclpy
from rclpy.node import Node
import csv
import os
import sys
import argparse
from geometry_msgs.msg import PoseWithCovarianceStamped

# CODE CREATED BY VaroAvila 

class AMCLPoseCSVWriterNode(Node):
    def __init__(self, topic_name, csv_file_path):
        super().__init__('amcl_pose_csv_writer_node')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            topic_name,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # CSV file setup
        self.csv_file_path = csv_file_path
        self.field_names = ['timestamp', 'position_x', 'position_y', 'position_z', 
                            'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w']
        self.csv_file = open(self.csv_file_path, 'w', newline='')
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.field_names, delimiter='=')
        self.writer.writeheader()

    def listener_callback(self, msg):
        # Extract data from msg and write to CSV
        data = {
            'timestamp': self.format_float(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9),
            'position_x': self.format_float(msg.pose.pose.position.x),
            'position_y': self.format_float(msg.pose.pose.position.y),
            'position_z': self.format_float(msg.pose.pose.position.z),
            'orientation_x': self.format_float(msg.pose.pose.orientation.x),
            'orientation_y': self.format_float(msg.pose.pose.orientation.y),
            'orientation_z': self.format_float(msg.pose.pose.orientation.z),
            'orientation_w': self.format_float(msg.pose.pose.orientation.w),
        }
        self.writer.writerow(data)

    def format_float(self, value):
        return "{:.3f}".format(value).replace('.', ',')

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    parser = argparse.ArgumentParser(description='AMCL Pose to CSV Writer.')
    parser.add_argument('topic_name', type=str, help='Name of the AMCL Pose ROS2 topic to subscribe to.')
    parser.add_argument('csv_file_path', type=str, help='Path to the output CSV file.')
    
    args = parser.parse_args()

    # Initialize ROS2
    rclpy.init(args=sys.argv)

    # Create and spin the node
    node = AMCLPoseCSVWriterNode(args.topic_name, args.csv_file_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
