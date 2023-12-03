# This node is to find objects in front view within 5 degrees

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

import numpy as np

class getObjectRange(Node):

    def __init__(self):
        super().__init__('get_object_range')

        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        self.lidar_subscriber = self.create_subscription(LaserScan,
                                            '/scan',
                                            self._lidar_callback,
                                            image_qos_profile)
        self.lidar_subscriber

        self.object_publisher = self.create_publisher(Point, '/object/position', 10)
        # Initialize
        self.object_position = Point()

        # Create a timer
        self.timer = self.create_timer(1.0/2.5, self.timer_callback)


    def timer_callback(self):
        self.object_publisher.publish(self.object_position)


    def _lidar_callback(self, LaserScan):
        self.object_position.y = 1.0

        # Find objects in front view within 5 degrees
        frontview_ind = int((np.pi/36) / LaserScan.angle_increment)

        ranges = np.array(LaserScan.ranges)
        ranges = np.where(ranges==np.NaN, 10, ranges)

        ranges_front = np.concatenate((ranges[:frontview_ind], ranges[-frontview_ind:]))

        index = (np.argpartition(ranges_front, 5))[:5]
        index = index.astype(int)

        # Measure the distance between robot and the object
        distance = float(np.median(ranges_front[index]))

        if (LaserScan.range_min <= distance and distance <= LaserScan.range_max):
            self.object_position.y = distance

        print("Object distance = ", self.object_position.y)


def main():
    rclpy.init()
    get_object_range = getObjectRange()
    rclpy.spin(get_object_range)
    get_object_range.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()