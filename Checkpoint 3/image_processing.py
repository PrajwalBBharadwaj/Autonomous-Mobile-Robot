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
    self.subscription = self.create_subscription(CompressedImage,'/camera/image/compressed', self.listener_callback, 5)
    self.subscription # prevent unused variable warning
    self.flag=False
    self.publisher_ = self.create_publisher(Point, 'ballcenteres', 5)
    timer_period = 0.1  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.ball_location=[]

    self.br = CvBridge()
    self.prevcircle=None

  def dist(self,x1,y1,x2,y2):
    return (x1-x2)**2+(y1-y2)**2

   
  def process_frame(self,frame):

    # Capture frame-by-frame
    
    cx,cy=(float('inf'),float('inf'))
    image_copy=frame.copy()
    self.flag=False

    hsv_image=cv2.cvtColor(image_copy,cv2.COLOR_BGR2HSV)
    lower_bound=np.array([16,97,175])
    upper_bound=np.array([179,255,255])
    color_mask=cv2.inRange(hsv_image,lower_bound,upper_bound)
    kernel=np.ones((5,5),np.uint8)
    mask=cv2.morphologyEx(cv2.morphologyEx(color_mask,cv2.MORPH_CLOSE,kernel),cv2.MORPH_OPEN,kernel)

    segmented_image=cv2.bitwise_and(image_copy,image_copy,mask)
    contours,_=cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    areas=[cv2.contourArea(c) for c in contours]

    try:
        index=np.argmax(areas)
        final_image=cv2.drawContours(segmented_image,contours[index],-1,(0,250,255),3)
        M=cv2.moments(contours[index])
        if M['m00']!=0:
            cx=M['m10']//M['m00']
            cy=M['m01']//M['m00']
    except:
        final_image=cv2.drawContours(segmented_image,contours,-1,(0,250,255),3)
        for contour in contours:
            M=cv2.moments(contour)
            if M['m00']!=0:
                cx=M['m10']//M['m00']
                cy=M['m01']//M['m00']
    if (cx,cy)!=(float('inf'),float('inf')):
        self.flag=True
        cx,cy=(cx,cy)
    
    self.ball_location = [cx, cy]
    print(self.ball_location)
    return segmented_image,self.ball_location
    

  def listener_callback(self, data):
    #self.get_logger().info('Receiving video frame')
    current_frame = self.br.compressed_imgmsg_to_cv2(data)
    processed_frame,self.ball_location=self.process_frame(current_frame)
    cv2.imshow("camera", processed_frame)
    cv2.waitKey(1)

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
