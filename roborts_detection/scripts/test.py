
import rospy
import roborts_msgs
from roborts_msgs.srv import FricWhl, FricWhlRequest, FricWhlResponse

def frictionWheelClient(req, res):
        rospy.wait_for_service("cmd_fric_wheel")
        try:
            fric_control = rospy.ServiceProxy('cmd_fic_wheel', FricWhl)
            fric_control(req, res)
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)



if __name__ == '__main__':
    while 1:
        fric_req = FricWhlRequest()
        fric_req.open = True
        frictionWheelClient(fric_req, FricWhlResponse())