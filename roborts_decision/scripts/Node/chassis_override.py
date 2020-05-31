import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
from Blackboard import Blackboard
def twist_callback(status):
    state[0] = status.data

state = ["DISABLE"]
rospy.init_node("pub_test")
pub  = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
pub2 = rospy.Publisher("/is_twist",String,queue_size=10)
sub = rospy.Subscriber("/cmd_twist",String,twist_callback)
rate = rospy.Rate(50)
vel = Twist()
vel.angular.z = 0
counter = 0
while not rospy.is_shutdown():
    if state[0] == "DISABLE" and counter ==0:
        pub2.publish(String("DISABLED"))
        pass    
    else:
        vel.angular.z = 20*math.sin(counter*math.pi/180) + 10*math.sin(6*counter*math.pi/180)
        counter = (counter + 4) % 360
        pub.publish(vel)
        pub2.publish(String("ENABLED"))
    rate.sleep()