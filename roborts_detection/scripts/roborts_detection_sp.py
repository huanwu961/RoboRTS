
import cv2 as cv
import numpy as np
import json
import roborts_detection_utils as utils
import mvsdk
#import torch

#ros related lib
import multiprocessing as mp
import rospy
import actionlib
from std_msgs.msg import String
from roborts_msgs.msg import ArmorDetectionAction,GimbalAngle,ArmorDetectionFeedback,ArmorDetectionResult, ShootState
from roborts_msgs.srv import FricWhl, FricWhlRequest, FricWhlResponse

class ArmorDetectionNode:
    # Initialize ros related as well as create python processes
    # Need to use shared Value and Array.
    # Need to define the semaphore mechanism for synchronization
    # The classifer need to be trained
    def __init__(self,debug=True):
        
        #init the detection_node
        self.enemy_info_pub_ = rospy.Publisher("cmd_gimbal_angle", GimbalAngle, queue_size=10)
        self.enemy_flag_pub_ = rospy.Publisher("Detection/flag", String, queue_size = 10)
        self.shoot_pub_ = rospy.Publisher("/cmd_shoot", ShootState, queue_size=10)
        rospy.init_node("armor_detection_node")

        self.fric_req = FricWhlRequest()
        self.fric_res = FricWhlResponse()

        # Multi-processing related definition
        self.parent_dt_pipe, self.frame_dt_pipe = mp.Pipe()
        #self.classifier = classifier
        self.status     = 'DETECT'
        self.as_        = actionlib.SimpleActionServer("armor_detection_node_action",
                                                       ArmorDetectionAction,
                                                       execute_cb=self.ActionCB,
                                                       auto_start = True)
                                                       
        self.initialized = False
        #rospy.init_node('armor_detection_node')
        # Publishers
        #self.enemy_info_pub_ = rospy.Publisher("cmd_gimbal_angle",GimbalAngle,100)
        ##self.enemy_flag_pub_ = rospy.Publisher("Detection/flag",String,100)
        # Initialize Algorithm related module:
        # Tracking
        with open('detector_params.json', 'r') as file:
            self.params = json.load(file)
           
        #self.fps_count = 0
        # Detection
        self.det_thresholdcolor = utils.BGRImageProc(color=self.params['color'],
                                                     threshs=self.params['bgr_threshs'],
                                                     enable_debug=debug)
        self.det_grey = utils.GrayImageProc()

        self.det_getlightbars   = utils.ScreenLightBars(thresh=self.params['bright_thresh'],
                                                        enable_debug=debug)

        self.det_filterlightbars= utils.FilterLightBars(light_max_aspect_ratio=self.params['light_max_aspect_ratio'],
                                                        light_min_area=self.params['light_min_area'],
                                                        light_max_area=self.params['light_max_area'],
                                                        light_min_aspect_ratio=self.params['light_min_aspect_ratio'],
                                                        light_max_angle_abs=self.params['light_max_angle_abs'],
                                                        enable_debug=debug)

        self.det_possiblearmors = utils.PossibleArmors(light_max_angle_diff=self.params['light_max_angle_diff'],
                                                       armor_max_aspect_ratio=self.params['armor_max_aspect_ratio'],
                                                       armor_min_area=self.params['armor_min_area'],
                                                       armor_max_pixel_val=self.params['armor_max_pixel_val'],
                                                       armor_center_min_color_average = self.params['armor_center_min_color_average'],
                                                       enable_debug=debug)

        '''self.det_filterarmors = utils.FilterArmors(armor_max_stddev=self.params['armor_max_stddev'],
                                                    armor_max_mean=self.params['armor_max_mean'],
                                                    enable_debug=debug) '''
        
        self.det_selectarmor  = utils.SelectFinalArmor(enable_debug=debug)

        self.det_armor2box = utils.Armor2Bbox

        #self.undetected_msg_published = False ---
        
        #self.detect_task = mp.Process(target=self.DetectionLoop)

        # Enemy pose information
        self.x = mp.Value('d',0.0)
        self.y = mp.Value('d',0.0)
        self.z = mp.Value('d',0.0)
        #self.gimbal_angle = GimbalAngle() ---
        self.gimbal_angle = GimbalAngle()
        #self.detected_flag= String()
        self.detected_flag = String()

        self.shoot = ShootState()
        # Open Camera
        DevList = mvsdk.CameraEnumerateDevice()
        if len(DevList)<1:
            raise IOError('No device found')
        self.camera = utils.Camera(30,DevList[0])

    # Action call back handler change flags
    # Periodically publish detection message for decision making
    def ActionCB(self,data):
        if not self.initialized:
            pass #rospy.logerror("Unable to initialize detection Node") ---
            return    
        cmd = data.command
        if cmd == 1:
            self.status = 'IDLE'
        elif cmd == 2:
            self.status = 'DETECT'
        elif cmd == 3:
            self.status = 'TRACK'
        else:
            raise NotImplementedError
        rospy.logdebug("status: " + self.status)
            
    # Send the message created armor detection
    # The gimbal angle is pre-set
    
    def frictionWheelClient(req, res):
        rospy.wait_for_service("cmd_fric_wheel")
        try:
            fric_control = rospy.ServiceProxy('cmd_fic_wheel', FricWhl)
            fric_control(req, res)
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)
    


    def SendMsg(self):
        
        if self.status == 'DETECT':
            #print("gimbal_angle: " + str(self.gimbal_angle.pitch_angle)+" "+ str(self.gimbal_angle.yaw_angle)+ " "+ str(self.gimbal_angle.pitch_mode) + " " + str(self.gimbal_angle.yaw_mode))
            self.gimbal_angle.yaw_mode = True
            self.gimbal_angle.pitch_mode = True
            self.enemy_info_pub_.publish(self.gimbal_angle)
            self.enemy_flag_pub_.publish(self.detected_flag)
        else:
            rospy.logdebug("Not detecting")

        '''
        if self.status == 'TRACK':
            self.enemy_info_pub_.publish(self.gimbal_angle)
            print("Status: ",self.status,self.gimbal_angle)
        # The gimbal will hold the last enemy occured place for 0.1 second
        if self.undetected_cnt.value < 10:
            self.detected_flag.data = 'YES'
            self.enemy_flag_pub_.publish(self.detected_flag)
            print(self.detected_flag)

        else:
            self.detected_flag.data = 'NO'
            self.enemy_flag_pub_.publish(self.detected_flag)
            print(self.detected_flag)
        '''



    # Process loop for tracking with kcf
    

    # Process loop for detection with constraint set and pass classifier
    # Need to be decomposed to multiple small tasks
    # Heavily based on the preprocess result.
    def DetectionLoop(self):
        self.camera.open()
        while not rospy.is_shutdown():
            
            frame           = self.camera.grab()
            cv.imshow("cam", frame)
            cv.waitKey(1)
            #print("frame:")
            #print(frame)

            grey            = self.det_grey(frame)
            #print(grey)

            thresh          = self.det_thresholdcolor(frame)
            #print("thresh:")
            #print(thresh)

            lightbars       = self.det_getlightbars(thresh, grey, frame)
            #print("lightbars:")
            #print(lightbars)

            rects           = self.det_filterlightbars(lightbars, frame)
            #print("rects: ")
            #print(rects)
            
            armors          = self.det_possiblearmors(rects,frame)
            #print("armors_1:")
            #print(armors)


#            armors          = self.det_filterarmors(armors, frame, classifier)
            #print("armors_2: ")
            #print(armors)
            
            target          = self.det_selectarmor(armors,frame)
            #print("target:")
            #print(target)

            bbox            = self.det_armor2box(target)

            #print("bbox")
            #print(bbox)

            #judge whethter the bbox is enemy's armor
            #print("bbox: ")
            #print(bbox)
            '''if bbox != (0, 0, 0, 0):
                state = self.classifier(frame, bbox)
            else:
                state = False'''

            if bbox == (0, 0, 0, 0):
                state = False
            else:
                state = True
            
            if state == True:
                self.status= 'DETECT'
                #if emeny's armor is detected, then set the enemy's pose
                self.x.value = bbox[0] +bbox[2]/2
                self.y.value = bbox[1] +bbox[3]/2
                
                #print("msg: " + str(self.x.value) + ' ' + str(self.y.value))

                #calculate the gimbal angle according to the enemy's pose
                self.gimbal_angle.pitch_angle,self.gimbal_angle.yaw_angle = utils.pixel2angle(self.params,
                                                                        self.x.value,
                                                                        self.y.value)

                #send the result
                self.SendMsg()
            else:
                self.status = 'NOT DETECT'
                self.detected_flag = 'NO'
                self.SendMsg()
                #self.fps_count += 1
                #rospy.Timer(rospy.Duration(1), self.fps_cb)

                print("Enemy is not detected!")

    '''def fps_cb(self):
        print("fps: " + str(self.fps_count))
        self.fps_count = 0'''
    def preprocess(self,frame):
        return frame

    def start(self):
        self.detect_task.start()
                
if __name__ == '__main__':
#    classifier = utils.CNNClassifier(path="../net_param.pth") # Need to train model
    node = ArmorDetectionNode() ### classifier deleted
    
    node.DetectionLoop()
    

    
