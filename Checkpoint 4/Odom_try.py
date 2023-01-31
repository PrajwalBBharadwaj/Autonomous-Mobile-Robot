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

class Navigate_Robot(Node):

    def __init__(self):
        super().__init__('navigate_robot')
        self.subscription = self.create_subscription(Odometry,'/odom',self.listener_callback, 10)
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
        self.get_logger().info('point x "%f"' % data.pose.pose.position.x)
        self.rotate = self.rotate_direction(data)

    def timer_callback(self):
        move_cmd = Twist()
        move_cmd.angular.x = float(0)
        move_cmd.angular.y = float(0)
        move_cmd.angular.z = float(self.rotate)
        self._vel_publish.publish(move_cmd)
        #self.get_logger().info('Publishing: "%s"' % move_cmd)
        #self.i += 1

    def update_Odometry(Odom):

        
        position = Odom.pose.pose.position
        
        #Orientation uses the quaternion aprametrization.
        #To get the angular position along the z-axis, the following equation is required.
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(Init_ang), np.sin(Init_ang)],[-np.sin(Init_ang), np.cos(Init_ang)]])        
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z

        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        

        #We subtract the initial values
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang
        
def main(args=None):

    rclpy.init(args=args)
    rotate_node = Navigate_Robot()
    rclpy.spin(navigate_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rotate_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
