from Blackboard import Blackboard
from BT import Action
import rospy
import tf
import PyConfig
import Helpers
# Messages
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Pose, Pose2D

class EnableShoot(Action):
    def __init__(self,name):
        super(EnableShoot,self).__init__(name)
        self.pub = rospy.Publisher("/shoot",String,queue_size=10)
        self.success_flag = True

    def Execute(self): # Beaware of pack loss
        try:
            self.pub.publish(String("ENABLE"))
            Blackboard.myrobot.enableShoot = True
        except:
            self.success_flag = False

    def Update(self):
        if self.success_flag == False:
            self._status = "FAILURE"
            return "FAILURE"
        self._status = "SUCCESS"
        return "SUCCESS"

    def Cleanup(self):
        self.success_flag = True

class DisableShoot(Action):
    def __init__(self,name):
        super(DisableShoot,self).__init__(name)
        self.pub = rospy.Publisher("/shoot",String,queue_size=10)
        self.success_flag = True
    
    def Execute(self):
        try:
            self.pub.publish(String("DISABLE"))
            Blackboard.myrobot.enableShoot = False
        except:
            self.success_flag = False
    
    def Update(self):
        if self.success_flag == False:
            self._status = "FAILURE"
            return "FAILURE"
        self._status = "SUCCESS"
        return "SUCCESS"

    def Cleanup(self):
        self.success_flag = True

class GOTO_BloodBuffer(Action):
    def __init__(self,name):
        super(GOTO_BloodBuffer,self).__init__(name)
        self.pub = rospy.Publisher("PyDecision/Goal",Pose,queue_size = 10)
        self.success_flag = True
        
    def Execute(self):
        bloodSupply = Blackboard.myrobot.bloodSupply
        if bloodSupply == 999:
            self.success_flag = False
            return
        else:
            pose = Pose()
            pose.position.x = PyConfig.BUFFERZONES[bloodSupply][0]
            pose.position.y = PyConfig.BUFFERZONES[bloodSupply][1]
            pose.position.z = 0
            quat = tf.transformations.quaternion_from_euler(0,0,PyConfig.BUFFERZONES[bloodSupply][2])
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            self.pub.publish(pose)

    def Update(self):
        if self.success_flag == False:
            self._status = "FAILURE"
        else:
            if Blackboard.myrobot.chassis == "IDLE":
                self._status = "RUNNING"
            else:
                self._status = Blackboard.myrobot.chassis
        return self._status
        
    def Cleanup(self):
        self.success_flag = True
        current_pose = Helpers.Pose2D2Pose(Blackboard.myrobot.pose)
        self.pub.publish(current_pose)

class GOTO_BulletBuffer(Action):
    def __init__(self,name):
        super(GOTO_BulletBuffer,self).__init__(name)
        self.pub = rospy.Publisher("PyDecision/Goal",Pose,queue_size = 10)
        self.success_flag = True
        
    def Execute(self):
        bulletSupply = Blackboard.myrobot.bulletSupply
        if bulletSupply == 999:
            self.success_flag = False
            return
        else:
            pose = Pose()
            pose.position.x = PyConfig.BUFFERZONES[bulletSupply][0]
            pose.position.y = PyConfig.BUFFERZONES[bulletSupply][1]
            pose.position.z = 0
            quat = tf.transformations.quaternion_from_euler(0,0,PyConfig.BUFFERZONES[bulletSupply][2])
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            self.pub.publish(pose)

    def Update(self):
        if self.success_flag == False:
            self._status = "FAILURE"
        else:
            if Blackboard.myrobot.chassis == "IDLE":
                self._status = "RUNNING"
            else:
                self._status = Blackboard.myrobot.chassis
        return self._status
        
    def Cleanup(self):
        self.success_flag = True
        current_pose = Helpers.Pose2D2Pose(Blackboard.myrobot.pose)
        self.pub.publish(current_pose)
        
class RandomTwist(Action):
    def __init__(self,name):
        super(RandomTwist,self).__init__(name)
        self.pub = rospy.Publisher("/cmd_twist",String,queue_size=10)

    def Execute(self):
        try:
            self.pub.publish(String("ENABLE"))
            self._status = "SUCCESS"
        except:
            self._status = "FAILURE"
    
    def Update(self):
        return self._status

class GOTO_FurthestDefence(Action):
    def __init__(self,name):
        super(GOTO_FurthestDefence,self).__init__(name)
        self.pub = rospy.Publisher("/PyDecision/Goal",Pose,queue_size = 10)
        self.success_flag = True
    def Execute(self):
        pose2d = Blackboard.enemy.pose
        dists = [Helpers.CalcDist(pose2d,d) for d in PyConfig.DEFENCEPOINTS]
        index = dists.index(max(dists))
        target_pose = PyConfig.DEFENCEPOINTS[index]
        target_pose = Helpers.Pose2D2Pose(target_pose)
        try:
            self.pub.publish(target_pose)
        except:
            self.success_flag = False
    
    def Update(self):
        if self.success_flag == False:
            self._status = "FAILURE"
        else:
            if Blackboard.myrobot.chassis == "IDLE":
                self._status = "RUNNING"
            else:
                self._status = Blackboard.myrobot.chassis
        return self._status
    
    def Cleanup(self):
        self.success_flag = True
        current_pose = Helpers.Pose2D2Pose(Blackboard.myrobot.pose)
        self.pub.publish(current_pose)

class Patrol(Action):
    def __init__(self,name):
        super(Patrol,self).__init__(name)
        self.pub = rospy.Publisher("/PyDecision/Goal",Pose,queue_size=10)
        self.current_i = 0
        self.first_run = True
        self.last_status    = "IDLE"
        self.current_status = "IDLE"

    def Execute(self):
        self.current_status = Blackboard.myrobot.chassis
        if self.first_run == True:
            self.first_run = False
            dists = [Helpers.CalcDist(Blackboard.myrobot.pose,i) for i in PyConfig.SEARCHZONES]
            self.current_i = dists.index(min(dists))
            next_goal = Helpers.Pose2D2Pose(PyConfig.SEARCHZONES[self.current_i])
            self.pub.publish(next_goal)
        elif self.last_status == "RUNNING" and self.current_status!= "RUNNING":
            self.current_i = (self.current_i + 1)%len(PyConfig.SEARCHZONES)
            next_goal = Helpers.Pose2D2Pose(PyConfig.SEARCHZONES[self.current_i])
            self.pub.publish(next_goal)
        else:
            pass # On The Way

    def Update(self):
        self._status = "RUNNING"
        return self._status

    def Cleanup(self):
        self.first_run = True
        self.current_i = 0
        current_pose = Helpers.Pose2D2Pose(Blackboard.myrobot.pose)
        self.pub.publish(current_pose)
        
class ChaseEnemy(Action):
    def __init__(self,name):
        super(ChaseEnemy,self).__init__(name)
        self.target_pose = Pose2D()
        self.first_run = True
        self.pub = rospy.Publisher("/PyDecision/Goal",Pose,queue_size=10)

    def Execute(self):
        if self.first_run == True:
            self.first_run = False
            self.target_pose = Blackboard.enemy.pose
        elif Helpers.CalcDist(self.target_pose,Blackboard.enemy.pose) > 1.5:
            self.target_pose = Blackboard.enemy.pose
        if(Helpers.CalcDist(Blackboard.myrobot.pose,self.target_pose)<=1.5):
            current_pose = Helpers.Pose2D2Pose(Blackboard.myrobot.pose)
            self.pub.publish(current_pose)
        else:
            target_pose = Helpers.Pose2D2Pose(self.target_pose)
            self.pub.publish(target_pose)

    def Update(self):
        self._status = "RUNNING"
        return self._status

    def Cleanup(self):
        self.first_run = True
        current_pose = Helpers.Pose2D2Pose(Blackboard.myrobot.pose)
        self.pub.publish(current_pose)
        