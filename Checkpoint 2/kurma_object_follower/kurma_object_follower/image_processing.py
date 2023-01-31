import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image,CompressedImage 
from cv_bridge import CvBridge 
import cv2
import numpy as np 
from std_msgs.msg import String
from geometry_msgs.msg import Point

class ImageSubscriber(Node):
  def __init__(self):
    super().__init__('image_subscriber')
    self.subscription = self.create_subscription(CompressedImage,'/camera/image/compressed', self.listener_callback, 10)
    self.subscription # prevent unused variable warning
    self.flag=False
    self.publisher_ = self.create_publisher(Point, 'ballcenteres', 10)
    timer_period = 0.5  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.ball_location=[]

    self.br = CvBridge()
    self.prevcircle=None

  def dist(self,x1,y1,x2,y2):
    return (x1-x2)**2+(y1-y2)**2

   
  def process_frame(self,frame):


    #frame=imutils.resize(frame,width=500)
    grayframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurframe = cv2.GaussianBlur(grayframe, (17,17), 1.5, 1.5)
    colorframe = cv2.cvtColor(grayframe, cv2.COLOR_GRAY2BGR)

    circles = cv2.HoughCircles(blurframe, cv2.HOUGH_GRADIENT, 0.5, 100, param1 = 100, param2 = 30, minRadius=0, maxRadius=300)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        chosen  = None
        for i in circles[0,:]:
            if chosen is None: chosen = i
            if self.prevcircle is not None:
                if self.dist(chosen[0],chosen[1],self.prevcircle[0],self.prevcircle[1]) == self.dist(i[0],i[1],self.prevcircle[0],self.prevcircle[1]):
                    chosen = i

        cv2.circle(frame, (chosen[0],chosen[1]),1,(0,100,100),3)    
        cv2.circle(frame, (chosen[0],chosen[1]),chosen[2],(255,0,255),3)

        #print(chosen[0],chosen[1])
        self.ball_location=[chosen[0],chosen[1]]
        height, width = frame.shape[:2]
        '''
        if width//2<chosen[0]:
            print("Move Right!")
        else:
            print("Move Left")
        self.prevcircle = chosen
        '''
        self.flag=True
    else:
      self.flag=False

    return frame,self.ball_location



  def listener_callback(self, data):
    #self.get_logger().info('Receiving video frame')
    current_frame = self.br.compressed_imgmsg_to_cv2(data)
    processed_frame,self.ball_location=self.process_frame(current_frame)
    #cv2.imshow("camera", processed_frame)
    #cv2.waitKey(1)

  def timer_callback(self):
    '''
    msg=String()
    msg.data=str(self.ball_location[0])+" "+str(self.ball_location[1])
    '''

    msg=Point()
    if self.flag==True:
      msg.x=float(self.ball_location[0])
      msg.y=float(self.ball_location[1])
      msg.z=0.0
    else:
      msg.x=float("inf")
      msg.y=float("inf")
      msg.z=float("inf")
    self.publisher_.publish(msg)
    #self.get_logger().info('Publishing: "%s"' % msg)

  
def main(args=None):
  
  rclpy.init(args=args)
  image_subscriber = ImageSubscriber()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
