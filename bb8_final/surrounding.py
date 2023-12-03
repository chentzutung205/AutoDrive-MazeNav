# This node is to find walls around the robot (range within 114 degrees)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

import numpy as np

class getWallRange(Node):

    def __init__(self):
        super().__init__('get_wall_range')

        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        self.lidar_subscriber = self.create_subscription(LaserScan,
                                            '/scan',
                                            self.lidar_callback,
                                            image_qos_profile)
        self.lidar_subscriber

        self.wall_publisher = self.create_publisher(Point, '/wall/position',10)
        # Initialize
        self.wall_position = Point()

        # Create a timer
        self.timer = self.create_timer(1.0/2.5, self.timer_callback)


    def timer_callback(self):
        self.wall_publisher.publish(self.wall_position)


    def lidar_callback(self, LaserScan):
        self.wall_position.y = 0.0
        self.wall_position.z = 0.0

        fov_ind = int(2.0 / LaserScan.angle_increment)

        ranges = np.array(LaserScan.ranges)
        ranges = np.where(ranges==np.NaN, 10, ranges)

        ranges_fov = np.concatenate((ranges[:fov_ind], ranges[-fov_ind:]))

        close_index = ((np.argpartition(ranges_fov, 6))[:6]).astype(int)

        wall_distance = float(np.median(ranges_fov[close_index]))

        indices = np.where(ranges == wall_distance)[0]
        if indices.size > 0:
            start_index = indices[0]
        else:
            start_index = 0
        
        wall_angle = start_index * LaserScan.angle_increment

        if (LaserScan.range_min <= wall_distance and wall_distance <= LaserScan.range_max):
            self.wall_position.y = wall_distance
            
            # Beware of angle! Convert clockwise 0 to 2pi lidar angles to clockwise -pi to pi lidar angles
            if wall_angle > np.pi:
                wall_angle = -(2*np.pi - wall_angle)
            else:
                wall_angle = wall_angle

            self.wall_position.z = wall_angle

        print("Wall angle = ", self.wall_position.z)
        print("Wall distance = ", self.wall_position.y)


def main():
    rclpy.init()
    get_wall_range = getWallRange()
    rclpy.spin(get_wall_range)
    get_wall_range.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()