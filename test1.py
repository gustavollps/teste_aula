import rospy
from geometry_msgs.msg import Twist


rospy.init_node('cmd_node')

def timerCallBack(event):
    msg = Twist()
    msg.linear.x = 1.0
    pub.publish(msg)
    

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
timer = rospy.Timer(rospy.Duration(0.05), timerCallBack)

rospy.spin()
