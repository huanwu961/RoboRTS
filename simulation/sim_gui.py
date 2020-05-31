import cv2 as cv
import numpy as np
import rospy
from std_msgs.msg import Int32,String
from geometry_msgs.msg import Pose2D,PoseStamped
from roborts_msgs.msg import BufferList
from math import pi
import random
from Blackboard import Blackboard
import Helpers
import time
import DemoBT
import BT

settle_flag = [False]
bufferzones = [[[0.60,2.60],[1.20,3.10]],
                       [[1.80,1.40],[2.35,2.10]],
                       [[4.04,3.80],[4.60,4.30]],
                       [[7.10,1.70],[7.70,2.20]],
                       [[5.90,2.80],[6.45,3.30]],
                       [[3.80,0.40],[4.40,0.90]]]

def printer(x,y):
    print("Received",x," ",y)
    print(cv.__version__)

def Nothing(x,y):
    print(x,y)
    pass

def IntPublisher(x,pub):
    x = Int32(x)
    pub.publish(x)
    
def StrPublisher(x,string,pub):
    string = String(string)
    pub.publish(string)

def map2world(x,y):
    x = (x)*(8.285/198)
    y = (116-y)*(4.685/112)
    return x,y

def world2map(x,y):
    x =int(x*(198.0/8.285))
    y =int(116 - y*(112.0/4.685))
    return x,y

def draw_target(event,x,y,flags,param,pub,map_display,map_display2):
    if event == cv.EVENT_LBUTTONDOWN:
        settle_flag[0]  = not settle_flag[0]
    if settle_flag[0] == True and event == cv.EVENT_MOUSEMOVE:
        map_display[:,:,:] = map[:,:,:]
        if x-6 <=5:
            left_x = 5
            right_x = 17
        elif x+6 > 198:
            right_x = 198
            left_x = 186
        else:
            left_x = x-6
            right_x = x+6
        if(y-7) <= 4:
            left_y = 4
            right_y = 18
        elif(y+7) > 112:
            right_y = 112
            left_y = 98
        else:
            left_y = y-7
            right_y= y+7
        cv.rectangle(map_display,(left_x,left_y),(right_x,right_y),(0,0,255),-1)
        map_display2[:,:,:]= map_display[:,:,:]
        robot_pose = Pose2D()
        robot_pose.x = (left_x+right_x)/2
        robot_pose.y = (left_y+right_y)/2
        robot_pose.x,robot_pose.y = map2world(robot_pose.x,robot_pose.y)
        robot_pose.theta = pi/2
        pub.publish(robot_pose)
        #print("Received!!!",robot_pose.x,robot_pose.y)

def draw_self(map_display2):
    map_display2[:,:,:] = map_display[:,:,:]
    cx = Blackboard.myrobot.pose.x
    cy = Blackboard.myrobot.pose.y
    #print(Blackboard.myrobot.pose)
    cx,cy = world2map(cx,cy)
    left_x = cx - 6
    left_y = cy - 7
    right_x= cx + 6
    right_y= cy + 7
    cv.rectangle(map_display2,(left_x,left_y),(right_x,right_y),(255,0,0),-1)
    ## Draw Bufferzone
    for i in range(len(bufferzones)):
        left_x,left_y   = world2map(bufferzones[i][0][0],bufferzones[i][1][1])
        right_x,right_y = world2map(bufferzones[i][1][0],bufferzones[i][0][1])
        if bufferzone.bloodbuffer == i:
            cv.rectangle(map_display2,(left_x,left_y),(right_x,right_y),(0,0,255),2)
        elif bufferzone.bulletbuffer == i:
            cv.rectangle(map_display2,(left_x,left_y),(right_x,right_y),(0,255,0),2)
        elif bufferzone.active_barrier[i] == 0:
            cv.rectangle(map_display2,(left_x,left_y),(right_x,right_y),(255,0,255),2)
        else:
            cv.rectangle(map_display2,(left_x,left_y),(right_x,right_y),(0,0,0),2)
    


class BufferZone(object):
    def __init__(self,pub):
        self.active_barrier = [1,1,1,1,1,1]
        self.buffers_status = [3,3,3,3,3,3]
        self.bloodbuffer = 999
        self.bulletbuffer= 999
        self.pub = pub
        self.bulletzone_pub = rospy.Publisher("award/bullet",Int32,queue_size=10)
        self.bloodzone_pub  = rospy.Publisher("award/blood",Int32, queue_size=10)

    def __call__(self,x,index): # 0-3 0: blood 1: bullet 2: no move 3: no shoot
        zones = [index,index+3]
        if x == 0:
            random.shuffle(zones)
            self.active_barrier[zones[0]] = 0
            self.active_barrier[zones[1]] = 1
            self.buffers_status[zones[0]] = 0
            self.buffers_status[zones[1]] = 0 
            self.bloodbuffer = zones[0]
            self.bulletbuffer = 999
            self.bloodzone_pub.publish(Int32(zones[0]))
        elif x==1:
            random.shuffle(zones)
            self.active_barrier[zones[0]] = 0
            self.active_barrier[zones[1]] = 1
            self.buffers_status[zones[0]] = 1
            self.buffers_status[zones[1]] = 1
            self.bulletbuffer = zones[0]
            self.bulletzone_pub.publish(Int32(zones[0]))
        else:
            self.active_barrier[zones[0]] = 1
            self.active_barrier[zones[1]] = 1
            self.buffers_status[zones[0]] = 3
            self.buffers_status[zones[1]] = 3
            if min(self.buffers_status) == 3:
                self.bloodzone_pub.publish(Int32(999))
                self.bloodbuffer = 999
                self.bulletzone_pub.publish(Int32(999))
                self.bulletbuffer = 999
        #print(self.buffers_status,self.bulletbuffer,self.bloodbuffer)
        bufferlist = BufferList(self.active_barrier)
        self.pub.publish(bufferlist)        

class RefereeSystem(object):
    def __init__(self):
        self.bulletzone_pub = rospy.Publisher("award/bullet",Int32,queue_size = 10)
        self.bloodzone_pub  = rospy.Publisher("award/blood",Int32,queue_size = 10)
        self.init_flag = True
        self.last_time = 0

    def checkBuffers(self):
        pose_id = self._checkpose()
        if pose_id == bufferzone.bulletbuffer:
            self.bulletzone_pub.publish(Int32(999))
            bufferzone.bulletbuffer = 999
            current_bullet = Blackboard.myrobot.bullet + 100
            Blackboard.myrobot.bullet = current_bullet
            cv.setTrackbarPos("Self Bullet","Game Panel",current_bullet)
        elif pose_id == bufferzone.bloodbuffer:
            self.bloodzone_pub.publish(Int32(999))
            bufferzone.bloodbuffer = 999
            current_blood = Blackboard.myrobot.blood + 200
            Blackboard.myrobot.blood = current_blood
            cv.setTrackbarPos("Self Blood","Game Panel",current_blood)

    def checkAttack(self):
        if self.init_flag == True:
                self.init_flag = False
                self.last_time = time.time()
        mypose = Blackboard.myrobot.pose
        enpose = Blackboard.enemy.pose
        dist = Helpers.CalcDist(mypose,enpose)
        isblock = Helpers.CheckBlock(mypose,enpose)
        if dist < 2 and dist > 1:
            if (time.time() - self.last_time) > 0.1:
                self.last_time = time.time()
                if not isblock:
                    if Blackboard.myrobot.bullet > 0 and Blackboard.myrobot.enableShoot:
                        Blackboard.enemy.blood -= 20
                        Blackboard.myrobot.bullet -= 1
                    if Blackboard.enemy.bullet > 0:
                        if Blackboard.myrobot.istwist ==True:
                            Blackboard.myrobot.blood -= 10
                        else:
                            Blackboard.myrobot.blood -= 20 
                        Blackboard.enemy.bullet -= 1
                    cv.setTrackbarPos("Target Blood","Game Panel",Blackboard.enemy.blood)
                    cv.setTrackbarPos("Target Bullet","Game Panel",Blackboard.enemy.bullet)
                    cv.setTrackbarPos("Self Blood","Game Panel",Blackboard.myrobot.blood)
                    cv.setTrackbarPos("Self Bullet","Game Panel",Blackboard.myrobot.bullet)
        elif dist <= 5 and dist >= 2:
            if (time.time() - self.last_time) > 0.1:
                self.last_time = time.time()
                if not isblock:
                    if Blackboard.myrobot.bullet > 0 and Blackboard.myrobot.enableShoot:
                        Blackboard.enemy.blood -= 10
                        Blackboard.myrobot.bullet -= 1
                    if Blackboard.enemy.bullet > 0:
                        if Blackboard.myrobot.istwist ==True:
                            Blackboard.myrobot.blood -= 5
                        else:
                            Blackboard.myrobot.blood -= 10 
                        Blackboard.enemy.bullet -= 1
                    cv.setTrackbarPos("Target Blood","Game Panel",Blackboard.enemy.blood)
                    cv.setTrackbarPos("Target Bullet","Game Panel",Blackboard.enemy.bullet)
                    cv.setTrackbarPos("Self Blood","Game Panel",Blackboard.myrobot.blood)
                    cv.setTrackbarPos("Self Bullet","Game Panel",Blackboard.myrobot.bullet)
        else:
            pass

    def checkWin(self):
        if Blackboard.myrobot.blood <=0:
            print("You Lose")
        elif Blackboard.enemy.blood <=0:
            print("You Win")

    def Update(self):
        self.checkWin()
        self.checkBuffers()
        self.checkAttack()
              
    def _checkpose(self):
        for i in range(len(bufferzones)):
            x = Blackboard.myrobot.pose.x
            y = Blackboard.myrobot.pose.y
            if (x >= bufferzones[i][0][0] and x <= bufferzones[i][1][0] and
                y >= bufferzones[i][0][1] and y <= bufferzones[i][1][1]):
                return i
            #print("x:",x,"y:",y)
        return -1




rospy.init_node("Test_Node")
bufferzone = BufferZone(rospy.Publisher("/buffer_sim_info",BufferList,queue_size=10))
self_blood = rospy.Publisher("/self/blood",Int32,queue_size=10)
self_bullet= rospy.Publisher("/self/bullet",Int32,queue_size=10)
target_bullet = rospy.Publisher("/target/bullet",Int32,queue_size=10)
target_blood  = rospy.Publisher("/target/blood",Int32,queue_size=10) 
game_startop = rospy.Publisher("game_info/start_stop",String,queue_size=10)
target_pose = rospy.Publisher("target/pose",Pose2D,queue_size = 10)


cv.namedWindow("Game Panel")
cv.createTrackbar("Self Blood",'Game Panel',0, 2000,lambda x:IntPublisher(x,self_blood))
cv.createTrackbar("Self Bullet",'Game Panel',0,500,lambda x:IntPublisher(x,self_bullet))
cv.createTrackbar("Target Blood",'Game Panel',0,2000,lambda x:IntPublisher(x,target_blood))
cv.createTrackbar('Target Bullet','Game Panel',0,500,lambda x:IntPublisher(x,target_bullet))
# For Buffer Zone config
cv.createTrackbar("Buffer Zone 1","Game Panel",0,3,lambda x: bufferzone(x,0))
cv.createTrackbar("Buffer Zone 2","Game Panel",0,3,lambda x: bufferzone(x,1))
cv.createTrackbar("Buffer Zone 3","Game Panel",0,3,lambda x: bufferzone(x,2))
cv.setTrackbarPos("Buffer Zone 1","Game Panel",3)
cv.setTrackbarPos("Buffer Zone 2","Game Panel",3)
cv.setTrackbarPos("Buffer Zone 3","Game Panel",3)
cv.setTrackbarPos("Self Blood","Game Panel",Blackboard.myrobot.blood)
cv.setTrackbarPos("Self Bullet","Game Panel",Blackboard.myrobot.bullet)
cv.setTrackbarPos("Target Blood","Game Panel",Blackboard.enemy.blood)
cv.setTrackbarPos("Target Bullet","Game Panel",Blackboard.enemy.bullet)

cv.createButton('Start',lambda x,y: StrPublisher(x,"START",game_startop))
cv.createButton('Stop',lambda x,y: StrPublisher(x,"STOP",game_startop))
cv.setMouseCallback('Game Panel',lambda a1,a2,a3,a4,a5: draw_target(a1,a2,a3,a4,a5,target_pose,map_display,map_display2))
map = cv.imread('icra2020.pgm')
map_display = map.copy()
map_display2= map.copy()
blackboard = Blackboard()
referee = RefereeSystem()
'''
def Main(data):
    
    cv.imshow('Game Panel',map_display2)
    referee.Update()
    draw_self(map_display2)
    if cv.waitKey(1) == ord("q"):
        rospy.signal_shutdown("Quit")
        cv.destroyAllWindows()
sub = rospy.Subscriber("/tick",Int32,Main)
rospy.spin()
'''
'''
try:
    thread.start_new_thread(main,())
    thread.start_new_thread(rospy.spin,())
except:
    print("Failed to start new thread")
'''
game_root = DemoBT.createNode()
game_root.getName(1)
while not(rospy.is_shutdown()):
    cv.imshow('Game Panel',map_display2)
    referee.Update()
    draw_self(map_display2)
    try:
        rospy.wait_for_message("/self/bullet",BufferList,0.01)
        rospy.wait_for_message("/self/blood",Int32,0.01)
        rospy.wait_for_message("/target/bullet",BufferList,0.01)
        rospy.wait_for_message("/target/blood",Int32,0.01)
        rospy.wait_for_message("/amcl_pose",PoseStamped,0.01)
        rospy.wait_for_message("/Actuator/Chassis",String,0.01)
        rospy.wait_for_message("/game_info/start_stop",String,0.01)
    except:
        pass
    game_root.OnTick()
    if cv.waitKey(1) == ord("q"):
        break
cv.destroyAllWindows()


