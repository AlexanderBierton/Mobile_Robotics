import rospy
import cv2
import numpy
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseActionResult
from sensor_msgs.msg import LaserScan
from actionlib_msgs.msg import GoalID
from cv_bridge import CvBridge, CvBridgeError
rospy.init_node('image_converter', anonymous=True)

class image_converter:
    
    colour_list = []
    
    col_red = False
    col_yel = False
    col_gre = False
    col_blu = False
    
    colour_reached = False
    
    new_position = True
    pos_reached = False
    start_spin = True
    col_found = False
    turning = False
    
    av_col = 0
    col_arr = 0
    pos_dist = 0
    pos_val = 0
    pos_z = 0
    now = time.time()
    goal_sets = [[2, -4, -0.1], [2, 0, 0.5], [-4.3, 1.5, 0.1], [1, 4, 0.5]]
    
    def __init__(self):
        cv2.namedWindow("Image window", 1)
        cv2.startWindowThread()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw", Image, self.callback)
        self.scan_sub = rospy.Subscriber("/turtlebot/scan", LaserScan, self.scan_callback)
        self.nav_res = rospy.Subscriber("turtlebot/move_base/feedback", MoveBaseActionFeedback, self.resultcallback)
        self.nav_pub = rospy.Publisher("/turtlebot/move_base_simple/goal", PoseStamped, queue_size=0)
        self.cncl_pub = rospy.Publisher("/turtlebot/move_base/cancel", GoalID, queue_size=0)
        self.res_sub = rospy.Subscriber("/turtlebot/move_base/result", MoveBaseActionResult, self.rescallback)
        self.pos_sub = rospy.Subscriber("/turtlebot/amcl_pose", PoseWithCovarianceStamped, self.speed_dist)
        


    
    
    def callback(self, data):
        pub = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size=0)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
        
        GI = GoalID()
        
        t = Twist()
        
        ##global variables
        global colour_list
        global col_red
        global col_yel
        global col_gre
        global col_blu
        global colour_reached        
        global av_col
        global col_arr
        
        bgr_thresh = cv2.inRange(cv_image,
                                 numpy.array((200, 230, 230)),
                                 numpy.array((255, 255, 255)))

        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv_thresh = cv2.inRange(hsv_img,
                                 numpy.array((90, 150, 0)),
                                 numpy.array((180, 250, 250)))
        
        """Yellow"""
        lower_yellow = numpy.array([30, 30, 30])
        upper_yellow = numpy.array([50, 255, 200])
        """   """
        """Red"""
        lower_red = numpy.array([0, 100, 100])
        upper_red = numpy.array([0, 255, 255])
        """     """
        """Green"""
        lower_green = numpy.array([50, 50, 50])
        upper_green = numpy.array([100, 255, 255])
        """    """
        """Blue"""
        lower_blue = numpy.array([100, 100, 100])
        upper_blue = numpy.array([150, 255, 250])
        """    """
        
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
        
        """All colour masks for rgb Camera"""
        yellow_mask = cv2.inRange(hsv_img, lower_yellow, upper_yellow)
        red_mask = cv2.inRange(hsv_img, lower_red, upper_red)
        green_mask = cv2.inRange(hsv_img, lower_green, upper_green)
        blue_mask = cv2.inRange(hsv_img, lower_blue, upper_blue)
        
        """mask which implements all colours masks"""
        mask = yellow_mask + red_mask + green_mask + blue_mask     
        
        #Remove found colours when destination has been reached#       
        for colours in self.colour_list:
            if (colours == "Red"):
                mask = mask - red_mask
            if (colours == "Yellow"):
                mask = mask - yellow_mask
            if (colours == "Green"):
                mask = mask - green_mask
            if (colours == "Blue"):
                mask = mask - blue_mask
        
        """values for height, width and depth of image"""
        h, w, d = cv_image.shape        
        search_top = h/2 - 10
        search_bot = h/2 + 10
        
        """Mask borders for isolated detection"""
        mask[0:269, 0:w] = 0
        mask[400:h, 0:w] = 0
        mask[0:h, 350:w] = 0
        mask[0:h, 0:300] = 0
        
        
        """var M for getting values of the coloured mask"""
        M = cv2.moments(mask)
        
        """Used if robot first finds colour and is unable to see it anymore after moving"""
        if M['m00'] == 0 and self.col_found == True:
            self.col_found = False
            self.new_position = False      
            
        """If the mask variable detects a colour the value will be greater than 0"""
        if M['m00'] > 0:
            self.start_spin = False
            print "Colour Found"
            self.col_found = True
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(cv_image, (cx, cy), 10, (0, 255, 255), -1)
            err = cx - w/2
            if self.colour_reached == False:
                if self.new_position == True:
                    self.cncl_pub.publish(GI)
                t.linear.x = 0.6
                t.angular.z = -float(err) /100
                pub.publish(t)            
            else:
                print "m00 ================== ", M['m00']
                if M['m00'] > 140000 and M['m00'] > 850000:
                    self.colour_check()
                self.colour_reached = False 
                if self.new_position == True:
                    self.new_position = False
          
        #masked image showing robotic thinking and for colour detection#
        masked = cv2.bitwise_and(cv_image, cv_image, mask = mask)
        amask = cv2.bitwise_and(cv_image, cv_image, mask = mask)
        
        #find average values for colours on row and columns#
        av_per_row = numpy.average(amask, axis=0)
        av_col = numpy.average(av_per_row, axis=0)
        
        #add values to global array and print#
        self.col_arr = numpy.array(av_col) 
        #####print self.col_arr
        
        print "Bool : ", self.new_position
        if self.new_position == False:
            print "New Position"
            self.positioning(self.pos_val)
        
        elif self.new_position == True and self.dist >= 4 and self.start_spin == False and self.turning == False:
            print "MAXIMUM OVERDRIVE"
            mo = Twist()
            mo.linear.x = 0.2
            pub.publish(mo)
        
        print self.turning
        
        #reset local av_col value print colour list and display image#
        av_col = 0
        #####print self.colour_list
        cv2.imshow("Image window", masked)
        
        if self.start_spin == False:
            self.now = time.time()
        
        if self.start_spin == True:
            future = self.now + 7
                
            if time.time() < future:
                M = cv2.moments(mask)
                t.angular.z = 1
                pub.publish(t)
                print M['m00']
                pass
            if time.time() >= future:
                print "new Bool"
                self.start_spin = False
                self.new_position = False
                
            else:
                t.angular.z = 0
    
    #called when robot has stopped near a colour#
    def colour_check(self):
        #Access Global Array#
        global col_arr
        #if the average is in range of these 
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
    
    #Callback for laserscan to stop robot collision#
    def scan_callback(self, data):
        global colour_reached
        range_ahead = data.ranges[len(data.ranges)/2]
        if range_ahead < 1:
            self.colour_reached = True
            print "Robot Stopped: Range Ahead = %0.1f" % range_ahead
            
    
    def positioning(self, value):
        p = PoseStamped()
        set_x = self.goal_sets[value] [0]
        set_y = self.goal_sets[value] [1]
        set_w = self.goal_sets[value] [2]
        
        print "Numbers = ", set_x, " and ", set_y
        p.header.frame_id = "/map"
        p.pose.position.x = set_x
        p.pose.position.y = set_y
        
        p.pose.orientation.w = set_w
        self.nav_pub.publish(p)
        
    
    def resultcallback(self, value):
        if value.status.status == 1:
            self.new_position = True
    
    def rescallback(self, value):
        if value.status.status == 4:
            print "failed pos : ", self.pos_val
            if self.pos_val <= len(self.goal_sets):
                self.pos_val = self.pos_val + 1
                self.pos_reached = True
                self.start_spin = True
            
        if value.status.status == 3:
            print "At Position"
            if self.pos_val <= len(self.goal_sets):
                self.pos_reached = True
                if len(self.colour_list) == 4:
                    rospy.signal_shutdown("All colours found")
                self.pos_val = self.pos_val + 1
                self.start_spin = True
                print "value : ", self.pos_val
            
    def speed_dist(self, coor):
        temp = coor.pose.pose.orientation.z
        if self.pos_z >= temp +- 0.6 or self.pos_z <= temp +- 0.6:
            self.turning = True
        else:
            self.turning = False
        self.pos_z = coor.pose.pose.orientation.z
        print coor.pose.pose.position.x
        tarx = self.goal_sets [self.pos_val] [0]
        tary = self.goal_sets [self.pos_val] [1]
        
        varx = coor.pose.pose.position.x
        vary = coor.pose.pose.position.y
        
        calx = tarx - varx
        caly = tary - vary
        
        square = (calx ** 2) + (caly ** 2)
        self.dist = numpy.sqrt(square)
        
        print "dist = ", self.dist
        
        
        
        
        
    
       

image_converter()

rospy.spin()
cv2.destroyAllWindows()
