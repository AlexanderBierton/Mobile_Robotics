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
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw", Image, self.callback)
       # self.wheel_sub = rospy.Subscriber("/wheel_vel_left", Float32, self.wheelcallback)

    def callback(self, data):
        pub = rospy.Publisher('/turtlebot/cmd_vel', Twist)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
        t = Twist()
        #t.angular.z = 1.0
        #pub.publish(t)
        bgr_thresh = cv2.inRange(cv_image,
                                 numpy.array((200, 230, 230)),
                                 numpy.array((255, 255, 255)))

        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv_thresh = cv2.inRange(hsv_img,
                                 numpy.array((90, 150, 0)),
                                 numpy.array((180, 250, 250)))

        print numpy.mean(hsv_img[:, :, 0])
        print numpy.mean(hsv_img[:, :, 1])
        print numpy.mean(hsv_img[:, :, 2])
        
        #####Yellow
        lower_yellow = numpy.array([30, 30, 30])
        upper_yellow = numpy.array([50, 255, 200])
        #####
        
        #####Red
        lower_red = numpy.array([0, 100, 100])
        upper_red = numpy.array([0, 255, 255])
        #####
        
        ####Green
        lower_green = numpy.array([50, 50, 50])
        upper_green = numpy.array([100, 255, 255])
        ####
        
        ####Blue
        lower_blue = numpy.array([100, 100, 100])
        upper_blue = numpy.array([150, 255, 250])
        ####
        
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
        yellow_mask = cv2.inRange(hsv_img, lower_yellow, upper_yellow)
        red_mask = cv2.inRange(hsv_img, lower_red, upper_red)
        green_mask = cv2.inRange(hsv_img, lower_green, upper_green)
        blue_mask = cv2.inRange(hsv_img, lower_blue, upper_blue)
        
        mask = yellow_mask + red_mask + green_mask + blue_mask        
        
        amask = cv2.bitwise_and(cv_image, cv_image, mask = mask)
        
        h, w, d = cv_image.shape        
        search_top = 3*h/4
        search_bot = 3*h/4 + 20
        print "h = ",h
        print "w = ",w
        print "d = ",d
        mask[0:250, 0:w] = 0
        mask[300:h, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(cv_image, (cx, cy), 10, (0, 255, 255), -1)
            err = cx - w/2
            t.linear.x = 0.2
            t.angular.z = -float(err) /100
            pub.publish(t)
        else:
            t.linear.x = 0.4
            #pub.publish(t)
            print "Nothing Found"
            
        masked = cv2.bitwise_and(cv_image, cv_image, mask = mask)
        m = numpy.mean(masked)
        #print "this is mean ", m
        cv2.imshow("Image window", masked)
        meanOut = rospy.Publisher("/colour_output", String)
        s = String()
        s.data = str(m)
        meanOut.publish(s)
        

image_converter()

rospy.spin()
cv2.destroyAllWindows()
