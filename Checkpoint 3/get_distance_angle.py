import rclpy, geometry_msgs.msg
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point,Pose2D
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import math

class get_dist_angle(Node):

    def __init__(self):
        super().__init__('get_dist_angle')
        qos_profile = QoSProfile(depth = 10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self.lidar_subscription = self.create_subscription(LaserScan,'scan',self.lidar_callback, qos_profile)
        self.lidar_subscription
        self.ballcenter_subscription = self.create_subscription(Point,'ballcenteres',self.image_callback, 10)
        self.ballcenter_subscription  # prevent unused variable warning


        self._dist_angle_publish = self.create_publisher(Pose2D, '/dist_angle', 5)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.ranges = []
        self.k=62.2/320
        self.distance=0.0
        self.theta=0.0

    def lidar_callback(self, data):
        #self.get_logger().info('point x "%f"' % data.x)
        self.ranges = data.ranges
        #print(data.ranges)

    def image_callback(self,data):
        ball_x=data.x 
        diff_pixel=ball_x-160
        self.theta=diff_pixel*self.k 
        #print("*****************")
        #print(self.theta)
        #print(self.ranges)
        if len(self.ranges)>0:
            if self.theta == float('inf'):
                self.distance = float('inf')
            else:
                self.distance=self.ranges[int(self.theta)]
            print(self.distance, self.theta)
            print("*****************")


    def timer_callback(self):
        pose = Pose2D()
        pose.x=float(self.distance)
        pose.y=0.0
        pose.theta=float(self.theta)
        self._dist_angle_publish.publish(pose)
        #self.get_logger().info('Publishing: "%s"' % move_cmd)
        #self.i += 1


def main(args=None):

    rclpy.init(args=args)
    rotate_node = get_dist_angle()
    rclpy.spin(rotate_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rotate_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
