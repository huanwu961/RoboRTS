import rospy
import roslib
from roborts_msgs.msg import BufferList

input_array = [1,1,1,1,1,1]

def getUserInput():
    print()
    input_str = raw_input("Get Barriers: ").split()
    for i in range(len(input_str)):
        input_array[i] = int(input_str[i])

def sendMsgInput(pub):
    current_msg = BufferList()
    current_msg.data = input_array
    pub.publish(current_msg)

if __name__ == "__main__":
    publisher = rospy.Publisher('/buffer_sim_info',BufferList, queue_size = 10)
    rospy.init_node("bufferzone_simulation")
    rate = rospy.Rate(10)
    while(not rospy.is_shutdown()):
        cmd = int(input("1: Reset Action;2: Continue: "))
        if(cmd==1):
            getUserInput()
        else:
            pass
        sendMsgInput(publisher)
        rate.sleep()
    
