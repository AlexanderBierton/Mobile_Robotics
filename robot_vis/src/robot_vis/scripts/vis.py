import rospy
import cv2
import numpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
rospy.init_node('image_converter', anonymous=True)

class image_converter:
    
    colour_list = []
    
    col_red = False
    col_yel = False
    col_gre = False
    col_blu = False
    
    colour_reached = False
    av_col = 0
    col_arr = 0
    
    def __init__(self):

        cv2.namedWindow("Image window", 1)
        cv2.startWindowThread()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw", Image, self.callback)
        self.scan_sub = rospy.Subscriber("/turtlebot/scan", LaserScan, self.scan_callback)
        #self.wheel_sub = rospy.Subscriber("/wheel_vel_left", Float32, self.wheelcallback)


    
    
    def callback(self, data):
        pub = rospy.Publisher('/turtlebot/cmd_vel', Twist)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
        t = Twist()
        l = LaserScan
        
        ##global variables
        global colour_list
        
        global col_red
        global col_yel
        global col_gre
        global col_blu
        
        global colour_reached        
        global av_col
        global col_arr
        
        #t.angular.z = 1.0
        #pub.publish(t)
        bgr_thresh = cv2.inRange(cv_image,
                                 numpy.array((200, 230, 230)),
                                 numpy.array((255, 255, 255)))

        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv_thresh = cv2.inRange(hsv_img,
                                 numpy.array((90, 150, 0)),
                                 numpy.array((180, 250, 250)))

        #print numpy.mean(hsv_img[:, :, 0])
        #print numpy.mean(hsv_img[:, :, 1])
        #print numpy.mean(hsv_img[:, :, 2])
        
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
        
        ##All colour masks for rgb Camera
        yellow_mask = cv2.inRange(hsv_img, lower_yellow, upper_yellow)
        red_mask = cv2.inRange(hsv_img, lower_red, upper_red)
        green_mask = cv2.inRange(hsv_img, lower_green, upper_green)
        blue_mask = cv2.inRange(hsv_img, lower_blue, upper_blue)
        
        ##mask which implements all colours masks
        mask = yellow_mask + red_mask + green_mask + blue_mask     

        ##Remove found colours when destination has been reached        
        
        for colours in self.colour_list:
            if (colours == "Red"):
                mask = mask - red_mask
            if (colours == "Yellow"):
                mask = mask - yellow_mask
            if (colours == "Green"):
                mask = mask - green_mask
            if (colours == "Blue"):
                mask = mask - blue_mask
        ##Average Colour filtering mask
        
        amask = cv2.bitwise_and(cv_image, cv_image, mask = mask)
        
        h, w, d = cv_image.shape        
        search_top = h/2 - 10
        search_bot = h/2 + 10
        #print "h = ",h
        #print "w = ",w
        #print "d = ",d
        mask[0:250, 0:w] = 0
        mask[300:h, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(cv_image, (cx, cy), 10, (0, 255, 255), -1)
            err = cx - w/2
            
            if self.colour_reached == False:
                t.linear.x = 0.8
                t.angular.z = -float(err) /100
                pub.publish(t)
            
            else:
                self.colour_check()
                self.colour_reached = False
                        
        else:
            t.angular.z = 0.8
            pub.publish(t)
            print "Nothing Found"
            
        masked = cv2.bitwise_and(cv_image, cv_image, mask = mask)
        m = numpy.mean(masked)
        
        av_per_row = numpy.average(amask, axis=0)
        av_col = numpy.average(av_per_row, axis=0)
        
        self.col_arr = numpy.array(av_col) 
        print self.col_arr
        
            
        
        av_col = 0
        print self.colour_list
        cv2.imshow("Image window", masked)
        meanOut = rospy.Publisher("/colour_output", String)
                
        s = String()
        s.data = str(m)
        meanOut.publish(s)
        
        
        
    def colour_check(self):
        global col_arr
        if self.col_arr[2] > self.col_arr[0] and self.col_arr[2] > self.col_arr[1] and numpy.round(self.col_arr[2]) != numpy.round(self.col_arr[1]):
            print "Red"
            if self.col_red != True:
                self.colour_list.append("Red")                
            self.col_red = True
            
        elif self.col_arr[2] > self.col_arr[0] and self.col_arr[1] > self.col_arr[0] and numpy.round(self.col_arr[1]) == numpy.round(self.col_arr[2]):
            print "Yellow"
            if self.col_yel != True:
                self.colour_list.append("Yellow")                
            self.col_yel = True
            
        elif self.col_arr[1] > self.col_arr[0] and self.col_arr[1] > self.col_arr[2] and numpy.round(self.col_arr[1]) != numpy.round(self.col_arr[2]):
            print "Green"
            if self.col_gre != True:
                self.colour_list.append("Green")                
            self.col_gre = True            
            
        elif self.col_arr[0] > self.col_arr[1] and self.col_arr[0] > self.col_arr[2]:
            print "Blue"
            if self.col_blu != True:
                self.colour_list.append("Blue")                
            self.col_blu = True
        
        
        
        
    def scan_callback(self, data):
        global colour_reached
        range_ahead = data.ranges[len(data.ranges)/2]
        print "range ahead = %0.1f" % range_ahead
        if range_ahead < 0.9:
            self.colour_reached = True
        
    
    

image_converter()

rospy.spin()
cv2.destroyAllWindows()
