import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np

class followSign(Node):

    def __init__(self):
        super().__init__('sign_follower')

        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        self._odom_subscriber = self.create_subscription(Odometry,
                                                         '/odom',
                                                         self.update_odometry,
                                                         image_qos_profile)
        self._odom_subscriber

        self._wall_subscriber = self.create_subscription(Point,
                                                         '/wall/position',
                                                         self.update_wall,
                                                         image_qos_profile)
        self._wall_subscriber
        self.wall = Point()

        self.object_subscriber = self.create_subscription(Point,
                                                         '/object/position',
                                                         self.update_object,
                                                         image_qos_profile)
        self.object_subscriber
        self.object = Point()

        self._sign_subscriber = self.create_subscription(String,
                                                         '/sign/label',
                                                         self.update_sign,
                                                         image_qos_profile)
        self._sign_subscriber

        self.vel_publisher = self.create_publisher(Twist,
                                                   '/cmd_vel',
                                                   10)
        self.velocity = Twist()

        self.vel_timer = self.create_timer(1.0/5.0, self.vel_timer_callback)

        # Initial odom position variables
        self.Init = True
        self.Init_ang = 0.0
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_pos.z = 0.0
        
        # Global position variables
        self.globalPos = Point()
        self.globalPos.x = 0.0
        self.globalPos.y = 0.0
        self.globalAng = 0.0
        self.prevAng = self.globalAng

        self.state = "JustGO"

        self.sign = None
        self.signlist = []

        self.distance_envelope = 0.02
        self.theta_envelope = 0.005
        
        # k-1 position variable for Derivative Controller
        self.old_position = Point()
        
        # P-controller gains
        self.distance_kp = 0.5
        self.theta_kp = 3.0
        self.lin_limit = 0.15
        self.ang_limit = 0.35

        # Counting how many times in LOST state loop
        self.times_lost = 0


    def angle_wrap(self, angle):
        angle = angle % (2*np.pi)
        if angle > np.pi:
            angle -= 2*np.pi
        return angle


    def exit(self):
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        return


    def find_wall(self):
        # Stop initially if not stopped
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        
        # If wall to the left, turns towards it until the object is within 5 degrees of the front of robot
        if self.wall.z > (10*self.theta_envelope):
            self.velocity.angular.z = min(self.ang_limit, self.theta_kp*self.wall.z)
        
        # If wall to the right, turns towards it until the object is within 5 degrees of the front of robot
        elif self.wall.z < (-10*self.theta_envelope):
            self.velocity.angular.z = max(-self.ang_limit, self.theta_kp*self.wall.z)
        
        # If wall straight ahead, go backward
        else:
            self.velocity.angular.z = 0.0
            self.velocity.linear.x = -0.06
        
        # Go backward until 0.44 m from the wall, then go to KNN state
        if self.wall.y > 0.44:
            self.times_lost += 1
            self.state = "KNN"


    def go_straight(self):
        if self.object.y == 0.0:
            self.object.y = 0.5

        # Go straight until a wall is found
        error = self.object.y - 0.43
        self.velocity.linear.x = min(error*self.distance_kp, self.lin_limit)
        self.velocity.angular.z = 0.0

        # Keep correcting angle
        self.goalAng = self.prevAng
        error = self.angle_wrap(self.goalAng-self.globalAng)
        
        if error > self.theta_envelope:
            self.velocity.angular.z = min(error*self.theta_kp, self.ang_limit)
        elif error < -self.theta_envelope:
            self.velocity.angular.z = max(error*self.theta_kp, -self.ang_limit)
        else:
            self.velocity.angular.z = 0.0

        # If an object is found, stop and detect sign
        if abs(self.object.y) < 0.45:
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = 0.0
            self.state = "KNN"
            return

    def knn(self):
        # Stop, detect sign, and move to rotate state
        self.confident = False
        
        # Stop
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        
        # Append detected sign here
        self.signlist.append(self.sign)
        self.prevAng = self.globalAng

        # When a sign is found correctly three times, robot is confident
        if len(self.signlist) >= 3:
            if self.signlist[-1] == self.signlist[-2] and self.signlist[-2] == self.signlist[-3]:
                self.confident = True
                sign = self.signlist[-1]
                self.signlist = []

        # Sign is right, left or reverse: accordingly update state
        if self.confident == True:
            if sign == "right":
                self.state = "TURN_R"
            elif sign == "left":
                self.state = "TURN_L"
            elif self.times_lost >= 2:
                self.state = "TURN_L"
            elif sign == "reverse":
                self.state = "UTURN"
            elif sign == "stop":
                self.state = "GOAL"
            else:
                self.state = "LOST"

        if len(self.signlist) >= 10:
            self.state = "LOST"

    def turn_left(self):
        # reset LOST state counter
        self.times_lost = 0
        
        # define goal angle 90 deg to left and turn
        self.goalAng = self.prevAng + np.pi/2
        error = self.angle_wrap(self.goalAng-self.globalAng)
        if error > self.theta_envelope:
            self.velocity.angular.z = min(error*self.theta_kp, self.ang_limit)
        elif error < -self.theta_envelope:
            self.velocity.angular.z = max(error*self.theta_kp, -self.ang_limit)
        else:
            # turn complete now go straight
            self.velocity.angular.z = 0.0
            self.state = "JustGO"
            self.prevAng = self.globalAng

    def turn_right(self):
        # reset LOST state counter
        self.times_lost = 0
        # define goal angle 90 deg to right and turn
        self.goalAng = self.prevAng - np.pi/2
        error = self.angle_wrap(self.goalAng-self.globalAng)
        if error > self.theta_envelope:
            self.velocity.angular.z = min(error*self.theta_kp, self.ang_limit)
        elif error < -self.theta_envelope:
            self.velocity.angular.z = max(error*self.theta_kp, -self.ang_limit)
        else:
            #turn complete now go straight
            self.velocity.angular.z = 0.0
            self.state = "JustGO"
            self.prevAng = self.globalAng

    def turn_around(self):
        # reset LOST state counter
        self.times_lost = 0
        # define goal angle 180 deg to right and turn
        self.goalAng = self.prevAng + np.pi
        error = self.angle_wrap(self.goalAng-self.globalAng)
        if error > self.theta_envelope:
            self.velocity.angular.z = min(error*self.theta_kp, self.ang_limit)
        elif error < -self.theta_envelope:
            self.velocity.angular.z = max(error*self.theta_kp, -self.ang_limit)
        else:
            # turn complete now go straight
            self.velocity.angular.z = 0.0
            self.state = "JustGO"
            self.prevAng = self.globalAng

    def info_update(self):

        print("state: ", self.state)
        print("front wall distance: ", self.object.y)
        print("sign: ", self.sign)
        print("nearest wall distance: ", self.wall.y)


    def update_odometry(self, Odom):
        position = Odom.pose.pose.position

        # Orientation uses the quaternion aprametrization.
        # To get the angular position along the z-axis, the following equation is required.
        
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            # The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z
        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])

        # We subtract the initial values
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang

        self.globalAng = self.angle_wrap(self.globalAng)


    def update_wall(self, Point):
        self.wall = Point


    def update_object(self, Point):
        self.object = Point


    def update_sign(self, sign):
        self.sign = sign.data


    def vel_timer_callback(self):
        self.vel_publisher.publish(self.velocity)
        self.get_logger().info('Published a velocity: linear x=%1.3f, angular z=%1.3f' %(self.velocity.linear.x, self.velocity.angular.z))
        self.info_update()

        # State Machine
        if self.state == "JustGO":
            self.go_straight()
        elif self.state == "KNN":
            self.knn()
        elif self.state == "TURN_L":
            self.turn_left()
        elif self.state == "TURN_R":
            self.turn_right()
        elif self.state == "UTURN":
            self.turn_around()
        elif self.state == "LOST":
            self.find_wall()
        elif self.state == "DONE":
            self.exit()


def main():
    rclpy.init()
    sign_follower = followSign()
    rclpy.spin(sign_follower)
    sign_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()