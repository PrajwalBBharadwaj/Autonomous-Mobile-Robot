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
from rclpy.clock import Clock
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose2D
import time
import math

class Move_Robot(Node):

    def __init__(self):
        super().__init__('move_robot')
        self.subscription = self.create_subscription(Pose2D,'/dist_angle',self.listener_callback, 5)
        self.subscription  # prevent unused variable warning

        self._vel_publish = self.create_publisher(Twist,'/cmd_vel', 5)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.move =float()
        self.rotate = float()

        self.cum_error_theta = 0
        self.cum_error_x = 0

    def move_direction(self,data):

        if data.x == float('inf'):
            self.move, self.rotate = 0.0,0.0
            print('do Nothing')
            return self.move, self.rotate

        reference_distance = 0.5
        reference_angle = 0.0

        object_distance = data.x
        object_angle = data.theta

        print('Object distance',object_distance)
        print('Object_angle',object_angle)

        Kp_x = 0.15
        Ki_x = 0.00
        Kd_x = 0.000

        Kp_theta = 0.05
        Ki_theta = 0.000
        Kd_theta = 0.0000

        prev_time = time.time()
        elapsed_time = 0.1
        prev_error = 0

        dist_tolerance = 0.05
        angle_tolerance = 5
        
        if abs(reference_distance - object_distance) > dist_tolerance or abs(reference_angle - object_angle) > angle_tolerance:

            self.move = 0.0

            pid_output_x = self.PID_x(reference_distance, object_distance, Kp_x, Ki_x, Kd_x, elapsed_time)
            pid_output_theta = self.PID_theta(reference_angle, object_angle, Kp_theta, Ki_theta, Kd_theta, elapsed_time)

            if abs(reference_distance - object_distance) > dist_tolerance:
                self.move = pid_output_x

            self.rotate = pid_output_theta
            print('Self move',self.move)

            return self.move, self.rotate

        else: 
            return 0.0, 0.0

        print('Reached location!')


    def PID_x(self, reference, current_state, Kp, Ki, Kd, elapsed_time):

        error = reference - current_state
        cum_error_x_prev = self.cum_error_x
        self.cum_error_x = self.cum_error_x + error * elapsed_time
        rate_error = (self.cum_error_x - cum_error_x_prev)/ elapsed_time
        output = Kp * error + Ki * (self.cum_error_x) + Kd * (rate_error)

        return output


    def PID_theta(self, reference, current_state, Kp, Ki, Kd, elapsed_time):

        error = reference - current_state
        cum_error_theta_prev = self.cum_error_theta
        self.cum_error_theta = self.cum_error_theta + error * elapsed_time
        rate_error = (self.cum_error_theta - cum_error_theta_prev)/ elapsed_time
        output = Kp * error + Ki * (self.cum_error_theta) + Kd * (rate_error)

        return output

    def listener_callback(self, data):

        #self.get_logger().info('point x "%f"' % data.x)
        print('Listener Callback')
        self.move, self.rotate = self.move_direction(data)

    def timer_callback(self):
        move_cmd = Twist()
        #print('Self Move', self.move)
        #print('Self Rotate', self.rotate)
        move_cmd.linear.x = -float(self.move)
        move_cmd.angular.x = float(0)
        move_cmd.angular.y = float(0)
        move_cmd.angular.z = float(self.rotate)
        print('Final publish',move_cmd.linear.x, move_cmd.angular.z)
        print('/n')
        self._vel_publish.publish(move_cmd)
        #self.get_logger().info('Publishing: "%s"' % move_cmd)
        #self.i += 1


def main(args=None):

    rclpy.init(args=args)
    move_node = Move_Robot()
    rclpy.spin(move_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    move_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


    '''
         
    if abs(reference_angle - object_angle) > angle_tolerance:

        pid_output_theta = self.PID_theta(reference_angle, object_angle, Kp_theta, Ki_theta, Kd_theta, elapsed_time)

        self.move = 0.0
        self.rotate = pid_output_theta

        #self.move, self.rotate = float(0), float(pid_output_theta)
        #print('Self move',self.move)
        return self.move, self.rotate
    else:
        return 0.0, 0.0
    '''

        #self.move, self.rotate = 0.0,0.0
        #return self.move, self.rotate
        #print('Reached orientation!')
        