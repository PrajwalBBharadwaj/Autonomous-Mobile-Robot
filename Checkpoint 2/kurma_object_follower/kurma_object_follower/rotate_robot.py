# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy, geometry_msgs.msg
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point
import math

class Rotate_Robot(Node):

    def __init__(self):
        super().__init__('rotate_robot')
        self.subscription = self.create_subscription(Point,'ballcenteres',self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        self._vel_publish = self.create_publisher(Twist, '/cmd_vel', 5)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.rotate = float()

    def rotate_direction(self,data):

        if data.x==float('inf'):
            self.rotate = 0.0
            print("Do nothing")
        elif data.x < 150:     # assuming frame size = 480 x 640
            self.rotate = 
            print("left")

        elif data.x > 170:
            self.rotate = -0.1
            print("right")

        else:
            self.rotate = 0.0

        #self.get_logger().info('rotate output : "%f"'% self.rotate)

        return self.rotate

    def listener_callback(self, data):
        #self.get_logger().info('point x "%f"' % data.x)
        self.rotate = self.rotate_direction(data)

    def timer_callback(self):
        move_cmd = Twist()
        move_cmd.angular.x = float(0)
        move_cmd.angular.y = float(0)
        move_cmd.angular.z = float(self.rotate)
        self._vel_publish.publish(move_cmd)
        #self.get_logger().info('Publishing: "%s"' % move_cmd)
        #self.i += 1


def main(args=None):

    rclpy.init(args=args)
    rotate_node = Rotate_Robot()
    rclpy.spin(rotate_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rotate_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
