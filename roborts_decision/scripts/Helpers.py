import numpy as np
import math
from geometry_msgs.msg import Pose, Pose2D,PoseStamped
import tf
import PyConfig

def Pose2Pose2D(pose):
    pose2d = Pose2D()
    pose2d.x = pose.position.x
    pose2d.y = pose.position.y
    _,_,pose2d.theta = tf.transformations.euler_from_quaternion([pose.orientation.x,
                                                                pose.orientation.y,
                                                                pose.orientation.z,
                                                                pose.orientation.w])
    return pose2d

def PoseStamped2Pose2D(posestamped):
    pose2d = Pose2D()
    pose2d.x = posestamped.pose.position.x
    pose2d.y = posestamped.pose.position.y
    _,_,pose2d.theta = tf.transformations.euler_from_quaternion([posestamped.pose.orientation.x,
                                                                posestamped.pose.orientation.y,
                                                                posestamped.pose.orientation.z,
                                                                posestamped.pose.orientation.w])
    return pose2d

def Pose2D2Pose(pose2d):
    pose = Pose()
    pose.position.x = pose2d.x
    pose.position.y = pose2d.y
    pose.position.z = 0
    q = tf.transformations.quaternion_from_euler(0,0,pose2d.theta)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose

def CalcDist(pt1,pt2):
    return math.sqrt((pt1.x-pt2.x)**2 + (pt1.y-pt2.y)**2)

def cross(p1,p2,p3):
    x1=p2[0]-p1[0]
    y1=p2[1]-p1[1]
    x2=p3[0]-p1[0]
    y2=p3[1]-p1[1]
    return x1*y2-x2*y1     

def segment(p1,p2,p3,p4):
    if(max(p1[0],p2[0])>=min(p3[0],p4[0])
    and max(p3[0],p4[0])>=min(p1[0],p2[0])
    and max(p1[1],p2[1])>=min(p3[1],p4[1])
    and max(p3[1],p4[1])>=min(p1[1],p2[1])):
        if(cross(p1,p2,p3)*cross(p1,p2,p4)<=0  
        and cross(p3,p4,p1)*cross(p3,p4,p2)<=0):
            D=True
        else:
            D=False
    else:
      D=False
    return D

def CheckBlock(pt1,pt2): # sq = [[x1,y1],[x2,y2],[x3,y3],[x4,y4]]
  # step 1 check if end point is in the square
    # step 2 check if diagonal cross the segment
    pt1 = [pt1.x,pt1.y]
    pt2 = [pt2.x,pt2.y] 
    for sq in PyConfig.BARRIERS:
        if (segment(pt1,pt2,sq[0],sq[1])
        or segment(pt1,pt2,sq[0],sq[2])
        or segment(pt1,pt2,sq[0],sq[3])
        or segment(pt1,pt2,sq[1],sq[2])
        or segment(pt1,pt2,sq[1],sq[3])
        or segment(pt1,pt2,sq[2],sq[3])):
            return True
    return False