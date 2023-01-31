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
import math
import numpy as np

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point,Pose2D
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

#from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Navigate_Robot(Node):

    def __init__(self):
        super().__init__('navigate_robot')
        self.Init_pos = Odometry().pose.pose.position
        self.globalPos = Odometry().pose.pose.position

        qos_profile = QoSProfile(depth = 10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self.lidar_subscription = self.create_subscription(LaserScan,'scan',self.lidar_callback, qos_profile)
        self.lidar_subscription

        self.odom_subscription = self.create_subscription(Odometry,'/odom',self.odom_callback, 10)
        self.odom_subscription 
        self.Init = True

        self._vel_publish = self.create_publisher(Twist, '/cmd_vel', 5)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.move =float()
        self.rotate = float()

        self.cum_error_theta = 0
        self.cum_error_x = 0

        self.ranges = []
        self.obstacle_distance = None
        self.obstacle_theta = None

        self.goal=[1.5,0]
        self.laser_dist = float()
        self.front_obs_dist = float()
        self.side_obs_dist = float()
        self.min_dist = float()
        self.min_dist_angle = float()
        self.ranges_length = int()

    def lidar_callback(self, data):
        
        length = len(data.ranges)
        self.ranges_length = length
        self.ranges = data.ranges


        if len(self.ranges)>0:
            min_f = self.ranges[0]
            for i in range(1,20):
                if self.ranges[i] < min_f:
                    min_f = self.ranges[i]
                if self.ranges[length-i] < min_f:
                    min_f = self.ranges[length-i]
            self.front_obs_dist = min_f

            min_dist = 3
            for dist in self.ranges:
                if dist>=0 and dist<=3:
                    if dist < min_dist:
                        min_dist = dist
            min_index = np.where(np.array(self.ranges)==min_dist)[0][0]
            #print('Min Index',min_index)
            self.min_dist = min_dist
            self.min_dist_angle = min_index/self.ranges_length*360


    def odom_callback(self, data):
        self.update_Odometry(data)        
        self.move, self.rotate = self.avoid_obstacle_goto_goal()

    def timer_callback(self):
        
        move_cmd = Twist()
        move_cmd.angular.x = float()
        move_cmd.angular.y = float(0)
        move_cmd.angular.z = self.rotate
        move_cmd.linear.x = self.move
        move_cmd.linear.y = float(0)
        move_cmd.linear.z = float(0)
        self._vel_publish.publish(move_cmd)
        print('Published',move_cmd.linear.x, move_cmd.angular.z)

    def distance(self,x1,y1,x2,y2):
        return math.sqrt((x1-x2)**2+(y1-y2)**2)

    def angle(self,x1,y1,x2,y2):
        return math.atan2((y2-y1),(x2-x1))

    def avoid_obstacle_goto_goal(self):

        thr1 = 0.5
        thr2 = 0.4

        dist_to_goal = self.distance(self.goal[0],self.goal[1],self.globalPos.x,self.globalPos.y)
        angle_t = (self.angle(self.globalPos.x,self.globalPos.y,self.goal[0],self.goal[1]) - self.globalAng) 
        sin_t = math.sin(angle_t)
        cos_t = math.cos(angle_t)
        angle_to_goal = math.atan2(sin_t,cos_t)

        print('******')
        print('global x',self.globalPos.x,'Global y',self.globalPos.y)
        print('Dist to goal',dist_to_goal,'Angle to goal',angle_to_goal)
        print('Obs_dist',self.front_obs_dist, 'Min dist',self.min_dist,'Min angle',self.min_dist_angle)
        #print('Length',len(self.ranges))
        print('******')

        if abs(angle_to_goal) > 0.1 or dist_to_goal > 0.02 :

            move = 0.0
            rotate = 0.0         

            if self.front_obs_dist > thr1: 
                if self.min_dist > thr2:
                    print('Goto goal')
                    move = 0.07*dist_to_goal
                    rotate = 0.15*angle_to_goal
                else:
                    print('Follow wall')
                    move = 0.07
                    rotate = 0.1           
            else:
                print('Avoid obstacle')
                move = 0.0
                rotate = -0.1

            if dist_to_goal > 0.02:
                self.move = move
            self.rotate = rotate

        else: 
            return 0.0, 0.0

        return self.move, self.rotate

    def move_direction(self,data):

        reference_distance = 1.5
        reference_angle = 0.0

        goal_distance = data[0]
        goal_angle = data[2]

        print('Goal distance',goal_distance,'Goal angle',goal_angle)
        print('Object distance',_distance,'Goal angle',goal_angle)

        Kp_x = 0.1
        Ki_x = 0.00
        Kd_x = 0.01

        Kp_theta = 0.05
        Ki_theta = 0.000
        Kd_theta = 0.001

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

    def update_Odometry(self, Odom):
  
        position = Odom.pose.pose.position
        
        #Orientation uses the quaternion aprametrization.
        #To get the angular position along the z-axis, the following equation is required.
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        

        if self.Init:
            print('Initial')
            
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init = False
            self.Init_ang = orientation
            
            self.globalAng = orientation
            self. Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.Init_pos.x = self.Mrot.item((0,0))*position.x + self.Mrot.item((0,1))*position.y
            self.Init_pos.y = self.Mrot.item((1,0))*position.x + self.Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z

    
        #We subtract the initial values
        self.globalPos.x = self.Mrot.item((0,0))*position.x + self.Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = self.Mrot.item((1,0))*position.x + self.Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang
        
def main(args=None):
    rclpy.init(args=args)
    navigate_node = Navigate_Robot()
    rclpy.spin(navigate_node)
    
    navigate_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()