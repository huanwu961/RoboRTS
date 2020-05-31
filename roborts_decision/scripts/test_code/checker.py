from geometry_msgs.msg import Pose2D
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

def check(pt1,pt2,sq): # sq = [[x1,y1],[x2,y2],[x3,y3],[x4,y4]]
  # step 1 check if end point is in the square
    # step 2 check if diagonal cross the segment
    pt1 = [pt1.x,pt1.y]
    pt2 = [pt2.x,pt2.y] 
    if (segment(pt1,pt2,sq[0],sq[1]) 
    or segment(pt1,pt2,sq[0],sq[2])
    or segment(pt1,pt2,sq[0],sq[3])
    or segment(pt1,pt2,sq[1],sq[2])
    or segment(pt1,pt2,sq[1],sq[3])
    or segment(pt1,pt2,sq[2],sq[3])):
        return True
    else:
        return False

pt1 = [0,3]
pt2 = [4,4]
sq = [[1,1],[0,2],[1,3],[2,2]]
print check(pt1,pt2,sq)
