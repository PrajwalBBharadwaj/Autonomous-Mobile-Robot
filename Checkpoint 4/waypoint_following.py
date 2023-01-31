import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point
import time
import numpy as np
import math

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point,Pose2D
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class MinimalSubscriber(Node):

	def __init__(self):
		super().__init__('minimal_subscriber')
		self.subscription = self.create_subscription(Odometry,'odom',self.listener_callback,10)


		self.Init_pos = Odometry().pose.pose.position
		self.globalPos = Odometry().pose.pose.position

		qos_profile = QoSProfile(depth = 10)
		qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
		qos_profile.durability = QoSDurabilityPolicy.VOLATILE

		self.lidar_subscription = self.create_subscription(LaserScan,'scan',self.lidar_callback, qos_profile)
		self.lidar_subscription

		self.subscription  # prevent unused variable warning
		self.Init=True
		self.goal=[[1.5,0.0],[1.5,1.5],[0.0,1.5]]
		self.waypoint=0 		#current waypoint number
		self.start=[0.0,0.0]
		self.distance_error=self.distance(self.goal[self.waypoint][0],self.goal[self.waypoint][1],self.start[0],self.start[1]) 	#initialising distance error
		self.angle_error=self.angle(self.start[0],self.start[1],self.goal[self.waypoint][0],self.goal[self.waypoint][1]) #initialising angle error
		self._vel_publish = self.create_publisher(Twist, '/cmd_vel', 5)
		timer_period = 0.2  # seconds
		self.timer = self.create_timer(timer_period, self.move_robot)
		self.globalPos=Odometry().pose.pose.position
		self.Init_pos=Odometry().pose.pose.position
		self.globalAng=0.0

		self.laser_dist = float()
		self.front_obs_dist = float()
		self.side_obs_dist = float()
		self.min_dist = float()
		self.min_dist_angle = float()
		self.ranges_length = int()

		self.thr1 = 0.3
		self.thr2 = 0.3	

	def distance(self,x1,y1,x2,y2):
		return math.sqrt((x1-x2)**2+(y1-y2)**2)

	def angle(self,x1,y1,x2,y2):
		out= np.arctan2((y2-y1),(x2-x1))

		return out


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


	def listener_callback(self, msg):		
		self.update_Odometry(msg)
		#print(round(self.globalPos.x,2),round(self.globalPos.y,2),round(self.globalAng,2))
		#exit()

	def wait_time(self,wait_time,curr_time):
		move_cmd = Twist()
		print("waiting 10 seconds")
		while time.time()-curr_time<wait_time:
			
			move_cmd.linear.x=0.0
			move_cmd.angular.z=0.0
			self._vel_publish.publish(move_cmd)	
		print("moving to next goal")	
		return



	def move_robot(self):
		move_cmd = Twist()
		self.distance_error=self.distance(self.globalPos.x,self.globalPos.y,self.goal[self.waypoint][0],self.goal[self.waypoint][1])
		rott=self.angle(self.globalPos.x,self.globalPos.y,self.goal[self.waypoint][0],self.goal[self.waypoint][1])
		self.angle_error=rott-self.globalAng
		self.angle_error=np.arctan2(math.sin(self.angle_error),math.cos(self.angle_error))
		
		print('Dist error',round(self.distance_error,2),'| Self Angle error',round(self.angle_error,2),'| Global Ang Error',round(rott,2))
		print('Global x',round(self.globalPos.x,2),' | Global y',round(self.globalPos.y,2),'| Global ang',round(self.globalAng,2),'| Goal',self.goal[self.waypoint])
		
		#v=0.15*self.angle_error
		if self.waypoint==0:
			dt=0.03
		elif self.waypoint==1:
			dt=0.08
		elif self.waypoint==2:
			dt=0.12
		if self.distance_error<dt:
			move=0.0
			rotate=0.0
			move_cmd.linear.x=move
			move_cmd.angular.z=rotate
			self._vel_publish.publish(move_cmd)
			if self.waypoint==len(self.goal)-1:
				print("All waypoints done")
				move_cmd.linear.x=0.0
				move_cmd.angular.z=0.0
				self._vel_publish.publish(move_cmd)
				exit()
			else:
				self.wait_time(2,time.time())

				#self.globalPos.x,self.globalPos.y=self.goal[self.waypoint][0],self.goal[self.waypoint][1]
				self.waypoint+=1			
		else:
			move=0
			rotate=0
			if self.front_obs_dist > self.thr1: 
				if self.min_dist > self.thr2:
					print('Goto goal')
					move = 0.15*self.distance_error
					rotate = 0.25*self.angle_error
				else:
					print('Follow wall')
					move = 0.07
					rotate = 0.15          
			else:
				print('Avoid obstacle')
				move = 0.0
				rotate = -0.15

			
			if move>0.2:
				move=0.2
			print('Linear_vel',move,'| Angular vel',rotate)
			move_cmd.linear.x=max(move,0.05)
			move_cmd.angular.z=rotate
			self._vel_publish.publish(move_cmd)
			print('************')



	def update_Odometry(self,Odom):
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
			Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
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

	minimal_subscriber = MinimalSubscriber()

	rclpy.spin(minimal_subscriber)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	minimal_subscriber.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()