import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

rospy.init_node('please work')

def callback(data):
    t = Twist()
    t.linear.x = 0.9 
    t.angular.z = 0.9
    pub.publish(t)
    pubb.publish(t)
    print data

sub = rospy.Subscriber("turtlebot_2/odom", Odometry, callback=callback)
pub = rospy.Publisher("/turtlebot_2/cmd_vel", Twist)
pubb = rospy.Publisher("/turtlebot_1/cmd_vel", Twist)

rospy.sleep(10)