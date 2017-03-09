import rospy
import cv2
import numpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
rospy.init_node('image_converter', anonymous=True)

class image_converter:
    
    def __init__(self):

        cv2.namedWindow("Image window", 1)
        cv2.startWindowThread()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/turtlebot_2/camera/rgb/image_raw", Image, self.callback)
        self.wheel_sub = rospy.Subscriber("/wheel_vel_left", Float32, self.wheelcallback)

    def callback(self, data):
        pub = rospy.Publisher('/turtlebot_2/cmd_vel', Twist)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
        t = Twist()
        t.angular.z = 0.2
        pub.publish(t)
        bgr_thresh = cv2.inRange(cv_image,
                                 numpy.array((200, 230, 230)),
                                 numpy.array((255, 255, 255)))

        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv_thresh = cv2.inRange(hsv_img,
                                 numpy.array((90, 150, 0)),
                                 numpy.array((180, 255, 255)))

        print numpy.mean(hsv_img[:, :, 0])
        print numpy.mean(hsv_img[:, :, 1])
        print numpy.mean(hsv_img[:, :, 2])
        
        lower_yellow = numpy.array([60, 10, 90])
        upper_yellow = numpy.array([100, 255, 255])
        
        bgr_contours, hierachy = cv2.findContours(bgr_thresh.copy(),
                                                  cv2.RETR_TREE,
                                                  cv2.CHAIN_APPROX_SIMPLE)

        hsv_contours, hierachy = cv2.findContours(hsv_thresh.copy(),
                                                  cv2.RETR_TREE,
                                                  cv2.CHAIN_APPROX_SIMPLE)
        for c in hsv_contours:
            a = cv2.contourArea(c)
            if a > 100.0:
                cv2.drawContours(cv_image, c, -1, (255, 0, 0))
        print '===='
        mask = cv2.inRange(hsv_img, lower_yellow, upper_yellow)
        
        h, w, d = cv_image.shape        
        #search_top = 3*h/4
        #search_bot = 3*h/4 + 20
        #mask[0:search_top, 0:w] = 0
        #mask[search_bot:h, 0:w] = 0
        #M = cv2.moments(mask)
        #if M['n00'] > 0:
            
        masked = cv2.bitwise_and(cv_image, cv_image, mask = mask)
        #m = numpy.mean(masked)
        #print "this is mean ", m
        cv2.imshow("Image window", masked)
        #meanOut = rospy.Publisher("/colour_output", String)
        #s = String()
        #s.data = str(m)
        #meanOut.publish(s)
    def wheelcallback(self, whldata):
        pub = rospy.Publisher('/turtlebot_2/cmd_vel', Twist)
        print "Yo mama ",whldata
        u = Twist()
        pub.publish(u)
        u.angular.z = whldata
        

image_converter()

rospy.spin()
cv2.destroyAllWindows()
