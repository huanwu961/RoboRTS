import Helpers
from geometry_msgs.msg import Pose, Pose2D
import math
import rospy
from BT import ConditionNode
from Blackboard import Blackboard

class IsBloodGT(ConditionNode):
    def __init__(self,name,val):
        super(IsBloodGT,self).__init__(name)
        self.value = val

    def Update(self):
        if Blackboard.myrobot.blood > self.value:
            self._status = "SUCCESS"
        else:
            self._status = "FAILURE"
        return self._status

class IsStart(ConditionNode):
    def Update(self):
        if Blackboard.game_status == "START":
            self._status = "SUCCESS"
        else:
            self._status = "FAILURE"
        return self._status

class IsBloodGTEnemy(ConditionNode):
    def Update(self):
        if Blackboard.myrobot.blood >= Blackboard.enemy.blood:
            self._status = "SUCCESS"
        else:
            self._status = "FAILURE"
        return self._status

class IsBulletGT(ConditionNode):
    def __init__(self,name,val):
        super(IsBulletGT,self).__init__(name)
        self.value = val

    def Update(self):
        if Blackboard.myrobot.bullet > self.value:
            self._status = "SUCCESS"
        else:
            self._status = "FAILURE"
        return self._status

class IsInRange(ConditionNode):
    def __init__(self,name,val):
        super(IsInRange,self).__init__(name)
        self.value = val

    def Update(self):
        if Helpers.CalcDist(Blackboard.enemy.pose,Blackboard.myrobot.pose) <= self.value:
            self._status = "SUCCESS"
        else:
            self._status = "FAILURE"
        return self._status

class IsBlocked(ConditionNode):
    def Update(self):
        if Helpers.CheckBlock(Blackboard.myrobot.pose,Blackboard.enemy.pose):
            self._status = "SUCCESS"
        else:
            self._status = "FAILURE"
        return self._status
