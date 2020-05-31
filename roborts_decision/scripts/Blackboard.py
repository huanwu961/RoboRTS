import rospy
import roslib
import tf

from std_msgs.msg import String,Int32
from roborts_msgs.msg import GimbalAngle, TwistAccel
from geometry_msgs.msg import PoseStamped, Pose2D
from math import pi

class Robot(object):
    def __init__(self,init_pose,init_blood,init_bullet):
        self.pose   = Pose2D(x=init_pose[0],y=init_pose[1],theta=init_pose[2])
        self.blood  = init_blood
        self.bullet = init_bullet
        self.chassis= "IDLE"
        self.istwist= False

class MyRobot(Robot):
    def __init__(self,init_pose,init_blood,init_bullet):
        super(MyRobot,self).__init__(init_pose,init_blood,init_bullet)
        self.enableShoot  = False # Refers to Gimble Task
        self.bloodSupply  = 999 # Indicate invalid
        self.bulletSupply = 999 # Indicate invalid

class EnemyRobot(Robot):
    def __init__(self,init_pose,init_blood,init_bullet,id):
        super(EnemyRobot,self).__init__(init_pose,init_blood,init_bullet)
        self.id = id

class Blackboard:
    game_status = "STOP"
    myrobot = MyRobot([1,1,pi],2000,50)
    enemy   = EnemyRobot([7.79,3.45,0.3],2000,50,1)
    # Can Add Second enemy
    mypose_sub         = None
    myblood_sub        = None
    mybullet_sub       = None
    mySupplyBlood_sub  = None
    mySupplyBullet_sub = None
    mychassis_sub      = None
    mytwist_sub        = None

    enemypose_sub      = None
    enemyblood_sub     = None
    enemybullet_sub    = None

    game_status_sub    = None
    def __init__(self):
        '''
        Create Subscribers attached named with
        On_xxx
        '''
        Blackboard.mypose_sub          = rospy.Subscriber("/amcl_pose",PoseStamped,self.on_mypose)
        Blackboard.myblood_sub         = rospy.Subscriber("/self/blood",Int32,self.on_myblood)
        Blackboard.mybullet_sub        = rospy.Subscriber("/self/bullet",Int32,self.on_mybullet)
        Blackboard.mySupplyBlood_sub   = rospy.Subscriber("/award/blood",Int32,self.on_bloodbuf)
        Blackboard.mySupplyBullet_sub  = rospy.Subscriber("/award/bullet",Int32,self.on_bulletbuf) 
        Blackboard.mychassis_sub       = rospy.Subscriber("Actuator/Chassis",String,self.on_chassis)
        Blackboard.mytwist_sub         = rospy.Subscriber("/is_twist",String,self.on_twist)

        Blackboard.enemypose_sub       = rospy.Subscriber("/target/pose",Pose2D,self.on_enemypose)
        Blackboard.enemyblood_sub      = rospy.Subscriber("/target/blood",Int32,self.on_enemyblood)
        Blackboard.enemybullet_sub     = rospy.Subscriber("/target/bullet",Int32,self.on_enemybullet)
        Blackboard.game_status_sub     = rospy.Subscriber("/game_info/start_stop",String,self.on_gamestatus)

    def on_mypose(self,posestp):
        pose2d = Pose2D()
        pose2d.x = posestp.pose.position.x
        pose2d.y = posestp.pose.position.y

        angles = tf.transformations.euler_from_quaternion([posestp.pose.orientation.x
                                                          ,posestp.pose.orientation.y
                                                          ,posestp.pose.orientation.z
                                                          ,posestp.pose.orientation.w])
        pose2d.theta = angles[2]
        Blackboard.myrobot.pose = pose2d
        #print Blackboard.myrobot.pose

    def on_myblood(self,blood):
        Blackboard.myrobot.blood = blood.data
        #print("My blood is: ",blood.data)

    def on_mybullet(self,bullet):
        Blackboard.myrobot.bullet= bullet.data
        #print("My bullet is: ",bullet.data)
    
    def on_bloodbuf(self,buf):
        Blackboard.myrobot.bloodSupply = buf.data

    def on_bulletbuf(self,buf):
        Blackboard.myrobot.bulletSupply= buf.data

    def on_enemyblood(self,blood):
        Blackboard.enemy.blood = blood.data

    def on_enemybullet(self,bullet):
        Blackboard.enemy.bullet= bullet.data

    def on_enemypose(self,pose2d):
        Blackboard.enemy.pose  = pose2d

    def on_gamestatus(self,status):
        Blackboard.game_status = status.data

    def on_chassis(self,status):
        Blackboard.myrobot.chassi = status.data

    def on_twist(self,status):
        if status.data == "ENABLED":
            Blackboard.myrobot.istwist = True
        else:
            Blackboard.myrobot.istwist = False

if __name__  == "__main__":
    rospy.init_node("blackboard_test")
    blackboard = Blackboard()
    rospy.spin()
    