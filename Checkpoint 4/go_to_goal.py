import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point

import numpy as np
import math

class MinimalSubscriber(Node):

	def __init__(self):
		super().__init__('minimal_subscriber')
		#qos_profile = QoSProfile(depth=10)
		#qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
		#qos_profile.durability = QoSDurabilityPolicy.VOLATILE
		self.subscription = self.create_subscription(Odometry,'odom',self.listener_callback,10)

		self.subscription  # prevent unused variable warning
		self.Init=True
		self.goal=[1.5,0]
		self.error=(self.goal[0]**2+self.goal[1]**2)**0.5
		self.angle_error=0#90*math.pi/180#1.5707
		self._vel_publish = self.create_publisher(Twist, '/cmd_vel', 5)
		timer_period = 0.5  # seconds
		self.timer = self.create_timer(timer_period, self.move_robot)

		self.globalPos=Odometry().pose.pose.position
		self.Init_pos=Odometry().pose.pose.position
		self.globalAng=0.0
		

	def distance(self,x1,y1,x2,y2):
		return math.sqrt((x1-x2)**2+(y1-y2)**2)

	def angle(self,x1,y1,x2,y2):
		return math.atan2((y2-y1),(x2-x1))

	def listener_callback(self, msg):		
		self.update_Odometry(msg)
		#print(round(self.globalPos.x,2),round(self.globalPos.y,2),round(self.globalAng,2))
		#exit()

	def move_robot(self):
		move_cmd = Twist()
		self.error=self.distance(self.globalPos.x,self.globalPos.y,self.goal[0],self.goal[1])
		self.angle_error=self.angle(self.globalPos.x,self.globalPos.y,self.goal[0],self.goal[1])-self.globalAng
		print(self.error,self.angle_error*180/math.pi)
		print(round(self.globalPos.x,2),round(self.globalPos.y,2),round(self.globalAng*180/math.pi,2))
		

		v=0.3*self.angle_error
		
		if self.error<0.05:
			u=0.0
			v=0.0
			print("stop!")
			move_cmd.linear.x=u
			move_cmd.angular.z=v
			self._vel_publish.publish(move_cmd)
			exit()
		else:
			u=0.08*self.error

		move_cmd.linear.x=max(u,0.01)
		print(move_cmd.linear.x,v)
		print("************************")
		move_cmd.angular.z=v
		self._vel_publish.publish(move_cmd)



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