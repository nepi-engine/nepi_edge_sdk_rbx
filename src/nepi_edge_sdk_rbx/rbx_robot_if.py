#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
import os
import time
import threading
import subprocess
import rospy
import numpy as np
import math
import tf
import random
import sys
import cv2

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Bool, String, Float32, Float64, Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, NavSatFix, BatteryState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from geographic_msgs.msg import GeoPoint, GeoPose, GeoPoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandHome
from pygeodesy.ellipsoidalKarney import LatLon
from cv_bridge import CvBridge

from nepi_ros_interfaces.msg import RBXStatus, AxisControls, RBXErrorBounds, RBXGotoErrors, RBXMotorControl, \
     RBXGotoPose, RBXGotoPosition, RBXGotoLocation
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest, RBXCapabilitiesQuery, RBXCapabilitiesQueryResponse, \
     NavPoseCapabilitiesQuery, NavPoseCapabilitiesQueryResponse

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_img
from nepi_edge_sdk_base import nepi_pc
from nepi_edge_sdk_base import nepi_nav

# ROS namespace setup
BASE_NAMESPACE = nepi_ros.get_base_namespace()

class ROSIDXSensorIF:
    # Default Global Values
    BAD_NAME_CHAR_LIST = [" ","/","'","-","$","#"]
    UPDATE_NAVPOSE_RATE_HZ = 10
    CHECK_SAVE_DATA_RATE_HZ = 40
    
    # Factory Control Values 
    GOTO_MAX_ERROR_M = 2.0 # Goal reached when all translation move errors are less than this value
    GOTO_MAX_ERROR_DEG = 2.0 # Goal reached when all rotation move errors are less than this value
    GOTO_STABILIZED_SEC = 1.0 # Window of time that setpoint error values must be good before proceeding
    CMD_TIMEOUT_SEC = 25 # Any action that changes 
    IMAGE_INPUT_TOPIC_NAME = "color_2d_image" # Partial or full ROS namespace string, "" for black image 
    

    NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()
    NAVPOSE_SERVICE_NAME = NEPI_BASE_NAMESPACE + "nav_pose_query"


    # Define class variables
    factory_device_name = None
    init_device_name = None
    factory_controls = None
 

    states = []
    modes = []
    actions = []
    data_products = ['image']

    settings_if = None
    save_data_if = None
    save_cfg_if = None
    
    rbx_status_info_pub_interval = float(1)/float(STATUS_UPDATE_RATE_HZ)
    check_save_data_interval_sec = float(1)/CHECK_SAVE_DATA_RATE_HZ
    update_navpose_interval_sec = float(1)/UPDATE_NAVPOSE_RATE_HZ


    def resetFactoryCb(self, msg):
        rospy.loginfo(msg)
        rospy.loginfo("Factory Resetting RBX Driver")
        self.resetFactory()

    def resetFactory(self):
        rospy.set_param('~rbx/device_name', self.factory_device_name)
        rospy.set_param('~rbx/max_error_m', GOTO_MAX_ERROR_M)
        rospy.set_param('~rbx/max_error_deg', GOTO_MAX_ERROR_DEG)
        rospy.set_param('~rbx/stabilized_sec', GOTO_STABILIZED_SEC)
        rospy.set_param('~rbx/cmd_timeout', CMD_TIMEOUT_SEC)
        rospy.set_param('~rbx/image_source', IMAGE_INPUT_TOPIC_NAME)   
        rospy.set_param('~rbx/status_image_overlay', False)  
        if self.setMotorControlRatio is not None:
          mc = RBXMotorControl()
          mc.speed_ratio = 0.0
          for i in range(len(self.getMotorControlRatios())):
            mc.motor_ind = i
            self.setMotorControlRatio(mc)
        self.settings_if.resetFactorySettings()
        self.updateFromParamServer()

    def updateDeviceNameCb(self, msg):
        #rospy.loginfo(msg)
        rospy.loginfo("Received Device Name update msg")
        new_device_name = msg.data
        self.updateDeviceName(new_device_name)

    def updateDeviceName(self, new_device_name):
        valid_name = True
        for char in self.BAD_NAME_CHAR_LIST:
            if new_device_name.find(char) != -1:
                valid_name = False
        if valid_name is False:
            self.update_error_msg("Received invalid device name update: " + new_device_name)
        else:
            rospy.set_param('~rbx/device_name', new_device_name)
        self.device_save_config_pub.publish(Empty())


    def resetDeviceNameCb(self,msg):
        #rospy.loginfo(msg)
        rospy.loginfo("Received Device Name reset msg")
        self.resetDeviceName()

    def resetDeviceName(self):
        rospy.set_param('~rbx/device_name', self.factory_device_name)
        self.device_save_config_pub.publish(Empty())

    def resetControlsCb(self, msg):
        rospy.loginfo(msg)
        rospy.loginfo("Resetting IDX Sensor Controls")
        self.resetControls()

    def resetControls(self):
        rospy.set_param('~rbx/device_name', self.init_device_name)
        rospy.set_param('~rbx/max_error_m', self.init_max_error_m)
        rospy.set_param('~rbx/max_error_deg', self.init_max_error_deg)
        rospy.set_param('~rbx/stabilized_sec', self.init_stabilized_sec)
        rospy.set_param('~rbx/cmd_timeout', self.init_cmd_timeout)
        rospy.set_param('~rbx/image_source', self.init_image_source)
        rospy.set_param('~rbx/status_image_overlay', self.init_status_image_overlay)
        self.updateFromParamServer()
    

    ##############################
    ### Update image source subscriber
    def image_subscriber_callback(self,img_msg):
        #Convert image from ros to cv2
        bridge = CvBridge()
        self.rbx_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")


    ##############################
    # RBX Settings Topic Callbacks

    # ToDo: Create a custom RBX status message
    ### Callback to set state
    def rbx_set_state_callback(self,state_msg):
        rospy.loginfo("*******************************")
        rospy.loginfo("Received set state message")
        rospy.loginfo(state_msg)
        state_val = state_msg.data
        self.rbx_set_state(state_val)

    ### Function to set state
    def rbx_set_state(self,new_state_ind):
        if new_state_ind < 0 or new_state_ind > (len(self.RBX_STATE_FUNCTIONS)-1):
            self.update_error_msg("No matching rbx state found")
        else:
            self.update_current_errors( [0,0,0,0,0,0,0] )
            self.rbx_status.process_current = self.states[new_state_ind]
            self.rbx_state_last = self.rbx_status.state
            rospy.loginfo("Waiting for rbx state " + self.states[new_state_ind] + " to set")
            rospy.loginfo("Current rbx state is " + self.states[self.rbx_status.state])
            self.setStateIndFunction(new_state_ind)
            self.update_prev_errors( [0,0,0,0,0,0,0] )
            self.rbx_status.process_last = self.states[new_state_ind]
            self.rbx_status.process_current = "None"
            time.sleep(1)
            if success:
                self.rbx_status.state = new_state_ind
        

    ### Callback to set mode
    def rbx_set_mode_callback(self,mode_msg):
        rospy.loginfo("*******************************")
        rospy.loginfo("Received set mode message")
        rospy.loginfo(mode_msg)
        mode_val = mode_msg.data
        self.rbx_set_mode(mode_val)

    ### Function to set mode
    def rbx_set_mode(self,new_mode_ind):
        if new_mode_ind < 0 or new_mode_ind > (len(self.RBX_MODE_FUNCTIONS)-1):
            self.update_error_msg("No matching rbx mode found")
        else:
            self.update_current_errors( [0,0,0,0,0,0,0] )
            self.rbx_status.process_current = self.modes[new_mode_ind]
            if self.modes[new_mode_ind] != "RESUME":
                self.rbx_mode_last = self.rbx_status.mode # Don't update last on resume
            rospy.loginfo("Setting rbx mode to : " + self.modes[new_mode_ind])
            rospy.loginfo("Calling rbx mode function: " + self.RBX_MODE_FUNCTIONS[new_mode_ind])
            self.setModeIndFunction(new_mode_ind)
            self.update_prev_errors( [0,0,0,0,0,0,0] )
            self.rbx_status.process_last = self.modes[new_mode_ind]
            self.rbx_status.process_current = "None"
            time.sleep(1)




    ### Callback to set manual motor control ratio
    def rbx_set_motor_control_callback(self,motor_msg):
        rospy.loginfo("*******************************")
        rospy.loginfo("Received set motor control ratio message")
        rospy.loginfo(motor_msg)
        new_motor_ctrl = mode_msg.data
        self.rbx_set_motor_control(new_motor_ctrl)

    ### Function to set motor control
    def rbx_set_motor_control(self,new_motor_ctrl):
        if self.manualControlsReadyFunction() is True:
            m_ind = new_motor_ctrl.motor_ind
            m_sr = new_motor_ctrl.speed_ratio
            m_len = len(self.getMotorControlRatios())
            if m_ind > (m_len -1):
                self.update_error_msg("New Motor Control Ind " + str(m_ind) + " is out of range")
            elif new_motor_ctrl < 0 or new_motor_ctrl > 1:
                self.update_error_msg("New Motor Control Speed Ratio " + str(m_sr) + " is out of range")
            elif self.setMotorControlFunction is not None:
                self.setMotorControlFunction(m_ind,m_sr)
                
        else:
            self.update_error_msg("Ignoring Set Motor Control msg, Manual Controls not Ready")     

        

    ### Callback to set home
    def rbx_set_home_current_callback(self,set_home_msg):
        rospy.loginfo("*******************************")
        rospy.loginfo("Received set home message")
        rospy.loginfo(set_home_msg)
        self.sethome_current()


    ### Callback to start rbx set goto goals process
    def rbx_set_goto_goals_callback(self,goto_goals_msg):
        rospy.loginfo("*******************************")
        rospy.loginfo("Received set goals message")
        rospy.loginfo(goto_goals_msg)
        rospy.set_param('~rbx/max_error_m', goto_goals_msg.max_distance_error_m)
        rospy.set_param('~rbx/max_error_deg', goto_goals_msg.max_rotation_error_deg)
        rospy.set_param('~rbx/stabilized_sec', goto_goals_msg.max_stabilize_time_s)

    ### Callback to set cmd timeout
    def rbx_set_cmd_timeout_callback(self,cmd_timeout_msg):
        rospy.loginfo("*******************************")
        rospy.loginfo("Received set timeout message")
        rospy.loginfo(cmd_timeout_msg)
        rospy.set_param('~rbx/cmd_timeout', self.init_cmd_timeout)


    ### Callback to image topic source
    def rbx_set_image_topic_callback(self,set_image_topic_msg):
        rospy.loginfo("*******************************")
        rospy.loginfo("Received set image topic message")
        rospy.loginfo(set_image_topic_msg)
        rospy.set_param('~rbx/image_source', self.init_image_source)


    ### Callback to image topic source
    def rbx_enable_image_overlay_callback(self,enable_msg):
        rospy.loginfo("*******************************")
        rospy.loginfo("Received enable image overlay message")
        rospy.loginfo(enable_msg)
        rospy.set_param('~rbx/status_image_overlay', self.init_status_image_overlay)

    ### Callback to set current process name
    def rbx_set_process_name_callback(self,set_process_name_msg):
        rospy.loginfo("*******************************")
        rospy.loginfo("Received set process name message")
        rospy.loginfo(set_process_name_msg)
        self.rbx_status.process_current = (set_process_name_msg.data)
    

    ##############################
    # RBX Control Topic Callbacks


    self.current_motor_controls

    ### Callback to start rbx go home
    def rbx_go_home_callback(self,home_msg):
        rospy.loginfo("*******************************")
        rospy.loginfo("Received go home message")
        rospy.loginfo(home_msg)
        if self.autonomousControlsReadyFunction() is True:
            if self.goHomeFunction is not None:
                self.rbx_status.process_current = "Go Home"
                self.rbx_cmd_success_current = False
                self.rbx_status.ready = False
                self.update_current_errors( [0,0,0,0,0,0,0] )
                self.rbx_cmd_success_current = self.goHomeFunction()
                time.sleep(1)
                self.rbx_status.process_last = "Go Home"
                self.rbx_status.process_current = "None"
                self.rbx_status.cmd_success = self.rbx_cmd_success_current
                time.sleep(1)
                self.rbx_status.ready = True
        else:
            self.update_error_msg("Ignoring Go command, Autononous Controls not Ready")

    ### Callback to start rbx stop
    def rbx_go_stop_callback(self,stop_msg):
        rospy.loginfo("*******************************")
        rospy.loginfo("Received go stop message")
        rospy.loginfo(stop_msg)
        if self.autonomousControlsReadyFunction() is True:
            if self.goStopFunction is not None:
                self.rbx_status.process_current = "Stop"
                self.rbx_cmd_success_current = False
                self.rbx_status.ready = False
                self.update_current_errors( [0,0,0,0,0,0,0] )
                self.rbx_cmd_success_current = self.goStopFunction()
                time.sleep(1)
                self.rbx_status.process_last = "Stop"
                self.rbx_status.process_current = "None"
                self.rbx_status.cmd_success = self.rbx_cmd_success_current
                time.sleep(1)
                self.rbx_status.ready = True
        else:
            self.update_error_msg("Ignoring Go command, Autononous Controls not Ready")

    ### Callback to execute action
    def rbx_go_action_callback(self,action_msg):
        rospy.loginfo("*******************************")
        rospy.loginfo("Received go action message")
        rospy.loginfo(action_msg)
        action_ind = action_msg.data
        if self.autonomousControlsReadyFunction() is True:
            if self.setActionIndFunction is not None:
                if action_ind < 0 or action_ind > (len(self.RBX_ACTION_FUNCTIONS)-1):
                    self.update_error_msg("No matching rbx action found")
                else:
                    if self.rbx_status.ready is False:
                        self.update_error_msg("Another GoTo Command Process is Active")
                        self.update_error_msg("Ignoring this Request")
                    else:
                        self.rbx_status.process_current = self.actions[action_ind]
                        self.rbx_status.ready = False
                        self.rbx_cmd_success_current = False
                        self.update_current_errors( [0,0,0,0,0,0,0] )
                        rospy.loginfo("Starting action: " + self.rbx_status.process_current)
                        time.sleep(1)
                        self.rbx_cmd_success_current = self.setActionIndFunction(action_ind)
                        rospy.loginfo("Finished action: " + self.rbx_status.process_current)
                        self.rbx_status.process_last = self.actions[action_ind]
                        self.rbx_status.process_current = "None"
                        self.rbx_status.cmd_success = self.rbx_cmd_success_current
                        time.sleep(1)
                        self.rbx_status.ready = True
        else:
            self.update_error_msg("Ignoring Go command, Autononous Controls not Ready")

    ### Callback to start rbx goto pose process
    def rbx_goto_pose_callback(self,pose_cmd_msg):
        rospy.loginfo("*******************************")
        rospy.loginfo("Recieved GoTo Pose Message")
        rospy.loginfo("")
        rospy.loginfo(pose_cmd_msg)
        if self.autonomousControlsReadyFunction() is True:
            setpoint_data=[pose_cmd_msg.roll_deg,pose_cmd_msg.pitch_deg,pose_cmd_msg.yaw_deg]
            if self.rbx_status.ready is False:
                self.update_error_msg("Ignoring GoTo POSE Request, Another GoTo Command Process is Active")
            else:
                self.rbx_status.process_current = "GoTo Pose"
                self.rbx_status.ready = False
                self.rbx_cmd_success_current = False
                self.update_current_errors( [0,0,0,0,0,0,0] )
                time.sleep(1)
                self.rbx_cmd_success_current = self.setpoint_attitude_ned(setpoint_data,self.rbx_status.cmd_timeout)
                self.rbx_status.process_last = "GoTo Pose"
                self.rbx_status.process_current = "None"
                self.rbx_status.cmd_success = self.rbx_cmd_success_current
                time.sleep(1)
                self.rbx_status.ready = True
        else:
        rospy.loginfo("Ignoring Go command, Autononous Controls not Ready")



    ### Callback to start rbx goto position process
    def rbx_goto_position_callback(self,position_cmd_msg):
        rospy.loginfo("*******************************")
        rospy.loginfo("Recieved GoTo Position Command Message")
        rospy.loginfo("")
        rospy.loginfo(position_cmd_msg)
        if self.rbx_status.in_manual_motor_control_mode is False:
            setpoint_data=[position_cmd_msg.x_meters,position_cmd_msg.y_meters,position_cmd_msg.z_meters,position_cmd_msg.yaw_deg]
            if self.rbx_status.ready is False:
                self.update_error_msg("Ignoring GoTo Position Request, Another GoTo Command Process is Active")
            else:
                self.rbx_status.process_current = "GoTo Position"
                self.rbx_status.ready = False
                self.rbx_cmd_success_current = False
                self.update_current_errors( [0,0,0,0,0,0,0] )
                time.sleep(1)
                self.rbx_status.ready = False
                self.rbx_cmd_success_current = self.setpoint_position_local_body(setpoint_data,self.rbx_status.cmd_timeout)
                self.rbx_status.process_last = "GoTo Position"
                self.rbx_status.process_current = "None"
                self.rbx_status.cmd_success = self.rbx_cmd_success_current
                time.sleep(1)
                self.rbx_status.ready = True
        else:
        rospy.loginfo("Ignoring Go command, Autononous Controls not Ready")

    ### Callback to start rbx goto location subscriber
    def rbx_goto_location_callback(self,location_cmd_msg):
        rospy.loginfo("*******************************")
        rospy.loginfo("Recieved GoTo Location Message")
        rospy.loginfo("")
        rospy.loginfo(location_cmd_msg)
        if self.autonomousControlsReadyFunction() is True:
            setpoint_data=[location_cmd_msg.lat,location_cmd_msg.long,location_cmd_msg.altitude_meters,location_cmd_msg.yaw_deg]
            if self.rbx_status.ready is False:
                self.update_error_msg("Ignoring GoTo Location Request, Another GoTo Command Process is Active")
            else:
                self.rbx_status.process_current = "GoTo Location"
                self.rbx_status.ready = False
                self.rbx_cmd_success_current = False
                self.update_current_errors( [0,0,0,0,0,0,0] )
                self.rbx_cmd_success_current = self.setpoint_location_global_wgs84(setpoint_data,self.rbx_status.cmd_timeout)
                self.rbx_status.process_last = "GoTo Location"
                self.rbx_status.process_current = "None"
                self.rbx_status.cmd_success = self.rbx_cmd_success_current
                time.sleep(1)
                self.rbx_status.ready = True
        else:
        self.update_error_msg("Ignoring Go command, Autononous Controls not Ready")

    # RBX Status Topic Publishers
    ### Callback for rbx status publisher
    def rbx_status_pub_callback(self,timer):
        self.rbx_status_publish()
    
    
    def rbx_status_publish(self):
        ## Update Status Image and Publish
        ## Update values from param server
        self.rbx_status.device_name = rospy.get_param('~rbx/device_name', self.init_device_name)
        error_bounds = RBXErrorBounds()
        error_bounds.max_distance_error_m = rospy.get_param('~rbx/max_error_m', self.init_max_error_m)
        error_bounds.max_rotation_error_deg = rospy.get_param('~rbx/max_error_deg', self.init_max_error_deg)
        error_bounds.max_stabilize_time_s = rospy.get_param('~rbx/stabilized_sec', self.init_stabilized_sec)
        self.rbx_status.error_bounds = error_bounds
        self.rbx_status.cmd_timeout = rospy.get_param('~rbx/cmd_timeout', self.init_cmd_timeout)
        self.rbx_status.image_source = rospy.get_param('~rbx/image_source', self.init_image_source)
        self.rbx_status.status_image_overlay = rospy.get_param('~rbx/status_image_overlay', self.init_status_image_overlay) 
        self.rbx_status.state = self.getStateIndFunction()
        self.rbx_status.mode = self.getModeIndFunction()
        self.rbx_battery = self.getBatteryPercent()
        ## Update Manual Motor Control Settings
        if self.getMotorControlRatios is not None:
            motor_controls_status_msg = self.get_motor_controls_status_msg(self.getMotorControlRatios())
        else:
            motor_controls_status_msg = self.get_motor_controls_status_msg([])
        self.rbx_status.current_motor_control_settings = motor_controls_status_msg
        # Update NavPose Info
        self.rbx_status.current_lat = self.current_location_wgs84_geo[0]
        self.rbx_status.current_long  = self.current_location_wgs84_geo[1]
        self.rbx_status.current_altitude  = self.current_location_wgs84_geo[2]
        self.rbx_status.current_heading = self.current_heading_deg
        self.rbx_status.current_roll = self.current_orientation_ned_degs[0]
        self.rbx_status.current_pitch  = self.current_orientation_ned_degs[1]
        self.rbx_status.current_yaw = self.current_orientation_ned_degs[2]

        # Create Status Info Text List
        status_info_msg = ["RBX Status"]
        if self.rbx_status.battery < 0.1:
            battery_string = "No Reading"
        else:
            battery_string = '%.2f' % self.rbx_status.battery
        status_info_msg.append("State: " + self.states[self.rbx_status.state])
        status_info_msg.append("Mode Current: " + self.modes[self.rbx_status.mode])
        status_info_msg.append("Mode Last: " + self.modes[self.rbx_mode_last])
        status_info_msg.append("Ready: " + str(self.rbx_status.ready))
        status_info_msg.append("")
        status_info_msg.append("Current Process: " + self.rbx_status.process_current)
        status_info_msg.append(" X,Y,Z Errors Meters: ")
        status_info_msg.append(" " + '%.2f' % self.rbx_status.errors_current.x_m + "," + '%.2f' % self.rbx_status.errors_current.y_m + "," + '%.2f' % self.rbx_status.errors_current.z_m)
        status_info_msg.append(" R,P,Y Errors Degrees: ")
        status_info_msg.append(" " + '%.2f' % self.rbx_status.errors_current.roll_deg + "," + '%.2f' % self.rbx_status.errors_current.pitch_deg + "," + '%.2f' % self.rbx_status.errors_current.yaw_deg)
        status_info_msg.append("")
        status_info_msg.append("Last Process: " + self.rbx_status.process_last)
        status_info_msg.append(" Success: " + str(self.rbx_status.cmd_success))
        status_info_msg.append(" X,Y,Z Errors Meters: ")
        status_info_msg.append(" " + '%.2f' % self.rbx_status.errors_prev.x_m + "," + '%.2f' % self.rbx_status.errors_prev.y_m + "," + '%.2f' % self.rbx_status.errors_prev.z_m)
        status_info_msg.append(" R,P,Y Errors Degrees: ")
        status_info_msg.append(" " + '%.2f' % self.rbx_status.errors_prev.roll_deg + "," + '%.2f' % self.rbx_status.errors_prev.pitch_deg + "," + '%.2f' % self.rbx_status.errors_prev.yaw_deg)
        status_info_msg.append("")
        if not rospy.is_shutdown():
            self.rbx_status_pub.publish(self.rbx_status)
            self.rbx_status_info_pub.publish(status_info_msg)
        # Overlay status info on image
        if self.rbx_status.status_image_overlay:
            box_x = 10
            box_y = 10
            box_w = 350
            box_h = 450
            # Add status box overlay
            cv2.rectangle(rbx_image, (box_x, box_y), (box_w, box_h), (255, 255, 255), -1)
            rbx_image = self.rbx_image # Initialize status image
            # Overlay Status Text List
            x=box_x+10 
            y=box_y+20
            for text in status_info_msg:
                self.status_text_overlay(rbx_image,text,x, y)
                y = y + 20
        # Create ROS Image message
        bridge = CvBridge()
        img_out_msg = bridge.cv2_to_imgmsg(rbx_image,"bgr8")#desired_encoding='passthrough')
        # Publish new image to ros
        if not rospy.is_shutdown():
            self.rbx_image_pub.publish(img_out_msg)
            # You can view the enhanced_2D_image topic at 
            # //192.168.179.103:9091/ in a connected web browser
        ## Update image source topic and subscriber if changed from last time.
        if self.rbx_status.image_source != self.rbx_image_source_last:
        #  If currently set, first unregister current image topic
            if self.rbx_image_source_last != "": 
                self.rbx_image_sub.unregister()
                time.sleep(1)
            # Try to find and subscribe to new image source topic
            #rospy.loginfo("Looking for topic: " + self.rbx_status.image_source)
            image_topic = nepi_ros.find_topic(self.rbx_status.image_source)
            #rospy.loginfo("Search returned: " + image_topic)
            if image_topic != "":  # If image topic exists subscribe
                self.rbx_image_sub = rospy.Subscriber(image_topic, Image, self.image_subscriber_callback, queue_size = 1)
                self.rbx_image_source_last = self.rbx_status.image_source
            else:
                self.update_error_msg("Unable to find image topic " + self.rbx_status.image_source)
            if self.rbx_status.image_source == "":
            self.rbx_image = self.rbx_image_blank # Set to blank image if source topic is cleared.
        
    ## Status Text Overlay Function
    def status_text_overlay(self,cv_image,status_text,x,y):
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (x,y)
        fontScale              = 0.5
        fontColor              = (0, 0, 0)
        thickness              = 1
        lineType               = 1
        cv2.putText(cv_image,status_text, 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            thickness,
            lineType)


    def updateFromParamServer(self):
        param_dict = rospy.get_param('~rbx', {})
        self.settings_if.updateFromParamServer()
   
    def provide_capabilities(self, _):
        return self.capabilities_report
    
    def provide_navpose_capabilities(self, _):
        return self.navpose_capabilities_report  

    def setCurrentAsDefault(self):
        pass # We only use the param server, no member variables to apply to param server
           
    def __init__(self, device_info, capSettings, 
                 factorySetting, settingUpdateFunction, getSettingsFunction,
                 axisControls,getBatteryPercentFunction,
                 states,getStateIndFunction,setStateIndFunction,
                 modes,getModeIndFunction,setModeIndFunction,
                 actions, setActionIndFunction,
                 manualControlsReadyFunction,autonomousControlsReadyFunction,
                 setHomeCurrentFunction=None,
                 goHomeFunction=None, goStopFunction=None, 
                 gotoPoseFunction=None, gotoPositionFunction=None, gotoLocationFunction=None,
                 motorControlFunction=None,
                 gpsTopic=None,odomTopic=None,headingTopic=None):
        
        
        self.node_name = device_info["node_name"]
        self.device_name = device_info["device_name"]
        self.identifier = device_info["identifier"]
        self.serial_num = device_info["serial_number"]
        self.hw_version = device_info["hw_version"]
        self.sw_version = device_info["sw_version"]

        self.factory_device_name = device_info["sensor_name"] + "_" + device_info["identifier"]
        self.init_device_name = rospy.get_param('~rbx/device_name', self.factory_device_name)
        rospy.set_param('~rbx/device_name', self.init_device_name)
        rospy.Subscriber('~reset_controls', Empty, self.resetControlsCb, queue_size=1) # start local callback

        self.states = states
        self.getStateFunction = getStateFunction
        self.setStateFunction = setStateFunction
        self.modes = modes
        self.getModeFunction = getModeFunction
        self.setModeFunction = setModeFunction
        self.actions = actions
        self.setActionFunction = setActionFunction
  

        # Create the CV bridge. Do this early so it can be used in the threading run() methods below 
        # TODO: Need one per image output type for thread safety?
        self.cv_bridge = CvBridge()


        # Define capabilities
        self.capabilities_report = RBXCapabilitiesQueryResponse()

        if axisControls == None:
          axis_controls = AxisControls()
          axis_controls.x = False
          axis_controls.y = False
          axis_controls.z = False
          axis_controls.roll = False
          axis_controls.pitch = False
          axis_controls.yaw = False
        self.capabilities_report.control_support = axisControls

        self.motorControlFunction = motorControlFunction
        if self.motorControlFunction is not None:
            self.capabilities_report.has_manual_mode = True
        else:
            self.capabilities_report.has_manual_mode = True
        
        self.capabilities_report.state_options = states
        self.capabilities_report.mode_options = modes
        self.capabilities_report.action_options = actions

        self.capabilities_report.data_products = str(self.data_products)


        self.nav_pose_capabilities_report = NavPoseCapabilitiesQueryResponse()

        self.nav_pose_capabilities_report.has_gps = GPSTopic is not None
        self.nav_pose_capabilities_report.has_orientation = OdomTopic is not None
        self.nav_pose_capabilities_report.has_heading = HeadingTopic is not None





    ## Initialize Class Variables
        self.update_navpose_interval = 0.1 # 10 Hz
        self.current_heading_deg = None
        self.current_orientation_enu_degs = None
        self.current_orientation_ned_degs = None
        self.current_position_enu_m = None
        self.current_position_ned_m = None
        self.current_location_amsl_geo = None
        self.current_location_wgs84_geo = None
        self.current_geoid_height_m = 0
        
        self.rbx_state_start = None
        self.rbx_state_last = None
        self.rbx_mode_start = None
        self.rbx_mode_last = None
        self.rbx_image_blank = np.zeros((350, 700, 3), dtype = np.uint8) # Empty Black Image
        self.rbx_image = self.rbx_image_blank
        self.rbx_image_source_last = ""
        
        self.mavlink_state = None
        
        
        if self.setMotorControlRatio is not None:
          mc = RBXMotorControl()
          mc.speed_ratio = 0.0
          for i in range(len(self.getMotorControlRatios())):
            mc.motor_ind = i
            self.setMotorControlRatio(mc)


        # Initialize IF and Status Message
        self.rbx_status=RBXStatus()
        self.rbx_status.connected = False
        self.rbx_status.serial_num = self.serial_num
        self.rbx_status.hw_version = self.hw_version
        self.rbx_status.sw_version = self.sw_version
        self.rbx_status.standby = False
        self.rbx_status.state = -999
        self.rbx_status.mode = -999
        self.rbx_status.process_current = "None"
        self.rbx_status.process_last = "None"
        self.rbx_status.ready = False
        self.rbx_status.battery = 0

        self.init_max_error_m = rospy.get_param('~rbx/max_error_m', GOTO_MAX_ERROR_M)
        rospy.set_param('~rbx/max_error_m', self.init_max_error_m)

        self.init_max_error_deg = rospy.get_param('~rbx/max_error_deg', GOTO_MAX_ERROR_DEG)
        rospy.set_param('~rbx/max_error_deg', self.init_max_error_deg)

        self.init_stabilized_sec = rospy.get_param('~rbx/stabilized_sec', GOTO_STABILIZED_SEC)
        rospy.set_param('~rbx/stabilized_sec', self.init_stabilized_sec)

        self.rbx_status.errors_current = RBXGotoErrors()
        self.rbx_status.errors_current.x_m = 0
        self.rbx_status.errors_current.y_m = 0
        self.rbx_status.errors_current.z_m = 0
        self.rbx_status.errors_current.heading_deg = 0
        self.rbx_status.errors_current.roll_deg = 0
        self.rbx_status.errors_current.pitch_deg = 0
        self.rbx_status.errors_current.yaw_deg = 0

        self.rbx_status.errors_prev = RBXGotoErrors()
        self.rbx_status.errors_prev.x_m = 0
        self.rbx_status.errors_prev.y_m = 0
        self.rbx_status.errors_prev.z_m = 0
        self.rbx_status.errors_prev.heading_deg = 0
        self.rbx_status.errors_prev.roll_deg = 0
        self.rbx_status.errors_prev.pitch_deg = 0
        self.rbx_status.errors_prev.yaw_deg = 0
        
        self.init_cmd_timeout = rospy.get_param('~rbx/cmd_timeout', CMD_TIMEOUT_SEC)
        rospy.set_param('~rbx/cmd_timeout', self.init_cmd_timeout)

        self.init_image_source = rospy.get_param('~rbx/image_source', IMAGE_INPUT_TOPIC_NAME)
        rospy.set_param('~rbx/image_source', self.init_image_source)   

        self.init_status_image_overlay = rospy.get_param('~rbx/status_image_overlay', False)
        rospy.set_param('~rbx/status_image_overlay', self.init_status_image_overlay)   

        self.rbx_status.cmd_success = False

        self.rbx_status.in_manual_motor_control_mode = self.manualControlsReadyFunction():
        self.rbx_status.in_autonomous_control_mode = self.autonomousControlsReadyFunction():

        if self.getMotorControlRatios is not None:
          motor_controls_status_msg = self.get_motor_controls_status_msg(self.getMotorControlRatios())
        else:
          motor_controls_status_msg = self.get_motor_controls_status_msg([])
        self.rbx_status.current_motor_control_settings = motor_controls_status_msg

        self.rbx_status.last_error_message = "" 


        ## Define NEPI Namespaces
        NEPI_SET_NAVPOSE_GPS_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_gps_fix_topic"
        NEPI_SET_NAVPOSE_HEADING_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_heading_topic"
        NEPI_SET_NAVPOSE_ORIENTATION_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/set_orientation_topic"
        NEPI_ENABLE_NAVPOSE_GPS_CLOCK_SYNC_TOPIC = NEPI_BASE_NAMESPACE + "nav_pose_mgr/enable_gps_clock_sync"

        ## Define Class Namespaces
        NEPI_NAV_SERVICE_NAME = NEPI_BASE_NAMESPACE + "nav_pose_query"
        NEPI_RBX_NAMESPACE = NEPI_BASE_NAMESPACE + self.node_name + "/rbx/"
        # NEPI RBX Driver Capabilities Publish Topics
        NEPI_RBX_CAPABILITIES_TOPIC = NEPI_RBX_NAMESPACE + "capabilities_query"
        NEPI_RBX_CAPABILITIES_NAVPOSE_TOPIC = NEPI_RBX_NAMESPACE + "navpose_query"
        # NEPI RBX Driver Status Publish Topics
        NEPI_RBX_STATUS_TOPIC = NEPI_RBX_NAMESPACE + "status" 
        NEPI_RBX_STATUS_INFO_TOPIC = NEPI_RBX_NAMESPACE + "status_info" 
        NEPI_RBX_IMAGE_TOPIC= NEPI_RBX_NAMESPACE + "image" # Image topic linked to driver
    

        # NEPI RBX Driver Settings Subscriber Topics
        NEPI_RBX_SET_STATE_TOPIC = NEPI_RBX_NAMESPACE + "set_state" # Int to Defined Dictionary self.states
        NEPI_RBX_SET_MODE_TOPIC = NEPI_RBX_NAMESPACE + "set_mode"  # Int to Defined Dictionary self.modes
        NEPI_RBX_SET_HOME_CURRENT_TOPIC = NEPI_RBX_NAMESPACE + "set_home_current" # Emplty
        NEPI_RBX_SET_GOTO_GOALS_TOPIC = NEPI_RBX_NAMESPACE + "set_goto_goals" # Float [Max_Meters,Max_Degrees,Stabilize_Time_Sec]
        NEPI_RBX_SET_CMD_TIMEOUT_TOPIC = NEPI_RBX_NAMESPACE + "set_cmd_timeout" # Int Seconds  - Any command that changes ready state
        NEPI_RBX_SET_IMAGE_TOPIC = NEPI_RBX_NAMESPACE + "set_image_topic" # full or partial ROS namespace
        NEPI_RBX_ENABLE_IMAGE_OVERLAY_TOPIC= NEPI_RBX_NAMESPACE + "enable_image_overlay" # Enable/Disable status info overlay on image
        NEPI_RBX_SET_PROCESS_NAME_TOPIC = NEPI_RBX_NAMESPACE + "set_process_name"  # string name of current process
        NEPI_RBX_SET_MOTOR_CONTROL_TOPIC = NEPI_RBX_NAMESPACE + "set_motor_control"  # Accepts update when Manual Motor Control Mode is Enabled

        # NEPI RBX Driver Control Subscriber Topics
        NEPI_RBX_GO_ACTION_TOPIC = NEPI_RBX_NAMESPACE + "go_action"  # Int to Defined Dictionary self.actions
        NEPI_RBX_GO_HOME_TOPIC = NEPI_RBX_NAMESPACE + "go_home" # Ignored if any active goto processes
        NEPI_RBX_GO_STOP_TOPIC = NEPI_RBX_NAMESPACE + "go_stop" # Aborts any active goto processes
        NEPI_RBX_GOTO_POSE_TOPIC = NEPI_RBX_NAMESPACE + "goto_pose" # Ignored if any active goto processes
        NEPI_RBX_GOTO_POSITION_TOPIC = NEPI_RBX_NAMESPACE + "goto_position" # Ignored if any active goto processes
        NEPI_RBX_GOTO_LOCATION_TOPIC = NEPI_RBX_NAMESPACE + "goto_location" # Ignored if any active goto processes

        ## Define NEPI Services Calls
        self.get_navpose_service = rospy.ServiceProxy(NEPI_NAV_SERVICE_NAME, NavPoseQuery)
        set_gps_pub = rospy.Publisher(NEPI_SET_NAVPOSE_GPS_TOPIC, String, queue_size=1)
        set_orientation_pub = rospy.Publisher(NEPI_SET_NAVPOSE_ORIENTATION_TOPIC, String, queue_size=1)
        set_heading_pub = rospy.Publisher(NEPI_SET_NAVPOSE_HEADING_TOPIC, String, queue_size=1)
        set_gps_timesync_pub = rospy.Publisher(NEPI_ENABLE_NAVPOSE_GPS_CLOCK_SYNC_TOPIC, Bool, queue_size=1)
    
        ## Create Class Sevices
        rospy.Service(NEPI_RBX_CAPABILITIES_TOPIC, RBXCapabilitiesQuery, self.rbx_capabilities_query_callback)
        rospy.Service(NEPI_RBX_CAPABILITIES_NAVPOSE_TOPIC, NavPoseCapabilitiesQuery, self.navpose_capabilities_query_callback)

        ## Create Class Publishers
        self.rbx_status_pub = rospy.Publisher(NEPI_RBX_STATUS_TOPIC, RBXStatus, queue_size=1)
        self.rbx_status_info_pub = rospy.Publisher(NEPI_RBX_STATUS_INFO_TOPIC, String, queue_size=1)
        self.rbx_image_pub = rospy.Publisher(NEPI_RBX_IMAGE_TOPIC, Image, queue_size=1)
                           
         ### Start RBX Settings Subscribe Topics
        rospy.Subscriber(NEPI_RBX_SET_STATE_TOPIC, UInt8, self.rbx_set_state_callback)
        rospy.Subscriber(NEPI_RBX_SET_MODE_TOPIC, UInt8, self.rbx_set_mode_callback)
        rospy.Subscriber(NEPI_RBX_SET_HOME_CURRENT_TOPIC, Empty, self.rbx_set_home_current_callback)
        rospy.Subscriber(NEPI_RBX_SET_GOTO_GOALS_TOPIC, RBXErrorBounds, self.rbx_set_goto_goals_callback)
        rospy.Subscriber(NEPI_RBX_SET_CMD_TIMEOUT_TOPIC, UInt32, self.rbx_set_cmd_timeout_callback)
        rospy.Subscriber(NEPI_RBX_SET_IMAGE_TOPIC, String, self.rbx_set_image_topic_callback)
        rospy.Subscriber(NEPI_RBX_ENABLE_IMAGE_OVERLAY_TOPIC, Bool, self.rbx_enable_image_overlay_callback)
        rospy.Subscriber(NEPI_RBX_SET_PROCESS_NAME_TOPIC, String, self.rbx_set_process_name_callback)

        ### Start RBX Control Subscribe Topics
        rospy.Subscriber(NEPI_RBX_SET_MOTOR_CONTROL_TOPIC, RBXMotorControl, self.rbx_set_motor_control_callback)
        rospy.Subscriber(NEPI_RBX_GO_ACTION_TOPIC, UInt8, self.rbx_go_action_callback)
        rospy.Subscriber(NEPI_RBX_GO_HOME_TOPIC, Empty, self.rbx_go_home_callback)
        rospy.Subscriber(NEPI_RBX_GO_STOP_TOPIC, Empty, self.rbx_go_stop_callback)
        rospy.Subscriber(NEPI_RBX_GOTO_POSE_TOPIC, RBXGotoPose, self.rbx_goto_pose_callback)
        rospy.Subscriber(NEPI_RBX_GOTO_POSITION_TOPIC, RBXGotoPosition, self.rbx_goto_position_callback)
        rospy.Subscriber(NEPI_RBX_GOTO_LOCATION_TOPIC, RBXGotoLocation, self.rbx_goto_location_callback)

        rospy.Subscriber('~rbx/reset_factory', Empty, self.resetFactoryCb, queue_size=1) # start local callback
        rospy.Subscriber('~rbx/update_device_name', String, self.updateDeviceNameCb, queue_size=1) # start local callbac
        rospy.Subscriber('~rbx/reset_device_name', Empty, self.resetDeviceNameCb, queue_size=1) # start local callback



        ## Start Node Processes
        # Start NavPose Data Updater
        rospy.Timer(rospy.Duration(self.update_navpose_interval), self.update_current_navpose_callback)
        # Start RBX Capabilities and Status Publishers
        rospy.Timer(rospy.Duration(self.rbx_status_info_pub_interval), self.rbx_status_pub_callback)
        # Connect to NEPI NavPose Solution
        # Set GPS Topic
        if gpsTopic is not None:
            set_gps_pub.publish(gpsTopic)
            rospy.loginfo("GPS Topic Set to: " + gpsTopic)
            # Sync NEPI clock to GPS timestamp
            set_gps_timesync_pub.publish(data=True)
            rospy.loginfo("Setup complete")
        # Set Orientation Topic
        if odomTopic is not None:
            set_orientation_pub.publish(odomTopic)
            rospy.loginfo("Orientation Topic Set to: " + odomTopic)
        # Set Heading Topic
        if headingTopic is not None:
            set_heading_pub.publish(headingTopic)
            rospy.loginfo("Heading Topic Set to: " + headingTopic)



        # Start interface classes and update
        self.settings_if = SettingsIF(capSettings, factorySettings, settingUpdateFunction, getSettingsFunction)
        self.save_data_if = SaveDataIF(data_product_names = self.data_products)
        self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.setCurrentAsDefault, paramsModifiedCallback=self.updateFromParamServer)



        





        # Start capabilities services
        rospy.Service('~idx/capabilities_query', IDXCapabilitiesQuery, self.provide_capabilities)
        rospy.Service('~idx/navpose_capabilities_query', NavPoseCapabilitiesQuery, self.provide_navpose_capabilities)

        
        # Launch the acquisition and saving threads
        if (self.image is not None):
            self.color_img_thread.start()
            self.color_2d_image = None
            self.color_2d_image_timestamp = None
            self.color_2d_image_lock = threading.Lock()
            rospy.Timer(rospy.Duration(self.check_save_data_interval_sec), self.saveColorImgThread)

        ## Initiation Complete
        rospy.loginfo("Initialization Complete")
        self.rbx_status.connected = True
        self.rbx_status.ready = True

  #######################
  # RBX IF Methods

  ### Setup a regular background navpose get and update navpose data
  def update_current_navpose_callback(self,timer):
    # Get current NEPI NavPose data from NEPI ROS nav_pose_query service call
    try:
      nav_pose_response = self.get_navpose_service(NavPoseQueryRequest())
      #rospy.loginfo(nav_pose_response)
      # Get current navpose
      current_navpose = nav_pose_response.nav_pose
      # Get current heading in degrees
      self.current_heading_deg = nepi_nav.get_navpose_heading_deg(nav_pose_response)
      # Get current orientation vector (roll, pitch, yaw) in degrees enu frame
      self.current_orientation_enu_degs = nepi_nav.get_navpose_orientation_enu_degs(nav_pose_response)
      # Get current orientation vector (roll, pitch, yaw) in degrees ned frame +-180
      self.current_orientation_ned_degs = nepi_nav.get_navpose_orientation_ned_degs(nav_pose_response)
      # Get current position vector (x, y, z) in meters enu frame
      self.current_position_enu_m = nepi_nav.get_navpose_position_enu_m(nav_pose_response)
      # Get current position vector (x, y, z) in meters ned frame
      self.current_position_ned_m = nepi_nav.get_navpose_position_ned_m(nav_pose_response)
      # Get current geoid hieght
      self.current_geoid_height_m =  nepi_nav.get_navpose_geoid_height(nav_pose_response)
      # Get current location vector (lat, long, alt) in geopoint data with WGS84 height
      self.current_location_wgs84_geo =  nepi_nav.get_navpose_location_wgs84_geo(nav_pose_response) 
      # Get current location vector (lat, long, alt) in geopoint data with AMSL height
      self.current_location_amsl_geo =  nepi_nav.get_navpose_location_amsl_geo(nav_pose_response)
##      rospy.loginfo("")
##      rospy.loginfo(self.current_geoid_height_m)
##      rospy.loginfo(self.current_location_wgs84_geo)
##      rospy.loginfo(self.current_location_amsl_geo)
    except Exception as e:
      self.update_error_msg("navpose service call failed: " + str(e))




 ### Function to set and check setpoint attitude NED command
  ###################################################
  # Input is [ROLL_NED_DEG, PITCH_NED_DEG, YEW_NED_DEGREES]
  # Converted to ENU befor sending message
  ###################################################
  def setpoint_attitude_ned(self,setpoint_attitude,timeout_sec=CMD_TIMEOUT_SEC):
    # setpoint_attitude is [ROLL_NED_DEG, PITCH_NED_DEG, YEW_NED_DEGREES]
    # Use value -999 to use current value
    cmd_success = True
    self.update_current_errors( [0,0,0,0,0,0,0] )
    rospy.loginfo("Starting Setpoint Attitude Create-Send-Check Process")
    ##############################################
    # Capture Current NavPose Data
    ##############################################
    start_orientation_ned_degs=list(self.current_orientation_ned_degs)
    rospy.loginfo('')
    rospy.loginfo("Attitude Current NED Degrees")
    rospy.loginfo(" Roll, Pitch, Yaw")
    rospy.loginfo(["%.2f" % start_orientation_ned_degs[0],"%.2f" % start_orientation_ned_degs[1],"%.2f" % start_orientation_ned_degs[2]])
    ##############################################
    # Condition Inputs
    ##############################################
    input_attitude_ned_degs = list(setpoint_attitude)
    rospy.loginfo('')
    rospy.loginfo("Attitude Input NED Degrees")
    rospy.loginfo(" Roll, Pitch, Yaw")
    rospy.loginfo(["%.2f" % input_attitude_ned_degs[0],"%.2f" % input_attitude_ned_degs[1],"%.2f" % input_attitude_ned_degs[2]])
    # Set new attitude in degs NED
    new_attitude_ned_degs=list(start_orientation_ned_degs) # Initialize with start values
    for ind in range(3): # Overwrite current with new if set and valid
      if setpoint_attitude[ind] != -999:
        new_attitude_ned_degs[ind]=setpoint_attitude[ind]
      # Condition to +-180 deg
      if new_attitude_ned_degs[ind] > 180:
        new_attitude_ned_degs[ind] = new_attitude_ned_degs[ind] - 360
    rospy.loginfo('')
    rospy.loginfo("Attitude Input Conditioned NED Degrees")
    rospy.loginfo(" Roll, Pitch, Yaw")
    rospy.loginfo(["%.2f" % new_attitude_ned_degs[0],"%.2f" % new_attitude_ned_degs[1],"%.2f" % new_attitude_ned_degs[2]])
    ##############################################
    # Convert NED attitude to Pose
    ##############################################
    # Convert to ROS ENU attitude degs and create ENU quaternion setpoint attitude goal
    yaw_enu_deg = nepi_nav.convert_yaw_ned2enu(new_attitude_ned_degs[2])
    new_attitude_enu_degs = [new_attitude_ned_degs[0],new_attitude_ned_degs[1],yaw_enu_deg]
    rospy.loginfo('')
    rospy.loginfo("Attitude Goal ENU Degrees")
    rospy.loginfo(" Roll, Pitch, Yaw")
    rospy.loginfo(["%.2f" % new_attitude_enu_degs[0],"%.2f" % new_attitude_enu_degs[1],"%.2f" % new_attitude_enu_degs[2]])
     ##############################################
    ## Send Setpoint Message and Check for Success
    ##############################################
    rospy.loginfo('')
    rospy.loginfo("Sending Setpoint Attitude Command at 50 Hz and")
    rospy.loginfo("Waiting for Attitude Setpoint to complete")
    setpoint_attitude_reached = False
    stabilize_timer=0
    timeout_timer = 0 # Initialize timeout timer
    print_timer = 0
    attitude_errors = [] # Initialize running list of errors
    while setpoint_attitude_reached is False and not rospy.is_shutdown():  # Wait for setpoint goal to be set
      if timeout_timer > timeout_sec:
        self.update_error_msg("Setpoint cmd timed out")
        cmd_success = False
        break
      time2sleep = 0.02
      time.sleep(time2sleep) # update setpoint position at 50 Hz
      stabilize_timer=stabilize_timer+time2sleep # Increment rospy.loginfo message timer
      timeout_timer = timeout_timer+time2sleep
      self.setGotoPoseFunction(new_attitude_enu_degs)
      # Calculate setpoint attitude errors
      cur_attitude_ned_degs = [self.current_orientation_ned_degs[0],self.current_orientation_ned_degs[1],self.current_orientation_ned_degs[2]]
      attitude_errors_degs = np.array(new_attitude_ned_degs) - np.array(cur_attitude_ned_degs)
      for ind in range(3):
        if input_attitude_ned_degs[ind] == -999.0: # Ignore error check if set to current
          attitude_errors_degs[ind]=0.0
      max_attutude_error_deg = max(abs(attitude_errors_degs))
      # Check for setpoint position local point goal
      if  setpoint_attitude_reached is False:
        if stabilize_timer > self.rbx_status.error_bounds.max_stabilize_time_s:
          max_attitude_errors = max(attitude_errors) # Get max from error window
          attitude_errors = [max_attutude_error_deg] # reset running list of errors
        print_timer = print_timer + time2sleep
        if print_timer > 1:
          print_timer = 0
          rospy.loginfo("")
          rospy.loginfo("Goto Pose Updates")
          # rospy.loginfo some information
          rospy.loginfo('')
          rospy.loginfo("Current Attitude NED Degrees")
          rospy.loginfo(" Roll, Pitch, Yaw")
          rospy.loginfo(["%.2f" % self.current_orientation_ned_degs[0],"%.2f" % self.current_orientation_ned_degs[1],"%.2f" % self.current_orientation_ned_degs[2]])
          rospy.loginfo('')
          rospy.loginfo("Current Goal NED Degrees")
          rospy.loginfo(["%.2f" % new_attitude_ned_degs[0],"%.2f" % new_attitude_ned_degs[1],"%.2f" % new_attitude_ned_degs[2]])
          rospy.loginfo('')
          rospy.loginfo("Current Attitude Errors")
          rospy.loginfo(["%.3f" % attitude_errors_degs[0],"%.3f" % attitude_errors_degs[1],"%.3f" % attitude_errors_degs[2]])
          rospy.loginfo("Max Error from Stabilized Check Window Meters")
          rospy.loginfo(["%.2f" % max_attitude_errors])
          if max_attitude_errors < self.rbx_status.error_bounds.max_rotation_error_deg:
            rospy.loginfo('')
            rospy.loginfo("Attitude Setpoint Reached")
            setpoint_attitude_reached = True
        else:
          attitude_errors.append(max_attutude_error_deg) # append last
      # Reset rospy.loginfo timer if past
      if stabilize_timer > GOTO_STABILIZED_SEC:
        stabilize_timer=0 # Reset rospy.loginfo timer
      self.update_current_errors( [0,0,0,0,attitude_errors_degs[0],attitude_errors_degs[1],attitude_errors_degs[2]]  )
    if cmd_success:
      rospy.loginfo("************************")
      rospy.loginfo("Setpoint Reached")
    self.update_current_errors( [0,0,0,0,0,0,0] )
    self.update_prev_errors( [0,0,0,0,attitude_errors_degs[0],attitude_errors_degs[1],attitude_errors_degs[2]] )
    return cmd_success
    


  ### Function to set and check setpoint position local body command
  ###################################################
  # Input is [X_BODY_METERS, Y_BODY_METERS, Z_BODY_METERS, YEW_BODY_DEGREES]
  # Converted to Local ENU Frame before sending
  # Local Body Position Setpoint Function use these body relative x,y,z,yaw conventions
  # x+ axis is forward
  # y+ axis is right
  # z+ axis is down
  # Only yaw orientation updated
  # yaw+ clockwise, yaw- counter clockwise from x axis (0 degrees faces x+ and rotates positive using right hand rule around z+ axis down)
  #####################################################
  def setpoint_position_local_body(self,setpoint_position,timeout_sec=CMD_TIMEOUT_SEC):
    # setpoint_position is [X_BODY_METERS, Y_BODY_METERS, Z_BODY_METERS, YEW_BODY_DEGREES]
    # use value 0 for no change
    cmd_success = True
    self.update_current_errors( [0,0,0,0,0,0,0] )
    rospy.loginfo('')
    rospy.loginfo("Starting Setpoint Position Local Create-Send-Check Process")
    ##############################################
    # Capture Current NavPose Data
    ##############################################
    start_geopoint_wgs84 = list(self.current_location_wgs84_geo)
    rospy.loginfo('')
    rospy.loginfo("Start Location WSG84 geopoint")
    rospy.loginfo(" Lat, Long, Alt")
    rospy.loginfo(["%.2f" % start_geopoint_wgs84[0],"%.2f" % start_geopoint_wgs84[1],"%.2f" % start_geopoint_wgs84[2]])
    start_position_ned_m = list(self.current_position_ned_m)
    rospy.loginfo('')
    rospy.loginfo("Start Position NED degs")
    rospy.loginfo(" X, Y, Z")
    rospy.loginfo(["%.2f" % start_position_ned_m[0],"%.2f" % start_position_ned_m[1],"%.2f" % start_position_ned_m[2]])   
    start_orientation_ned_degs=list(self.current_orientation_ned_degs)
    rospy.loginfo('')
    rospy.loginfo("Start Orientation NED degs")
    rospy.loginfo(" Roll, Pitch, Yaw")
    rospy.loginfo(["%.2f" % start_orientation_ned_degs[0],"%.2f" % start_orientation_ned_degs[1],"%.2f" % start_orientation_ned_degs[2]])
    rospy.loginfo('')
    start_yaw_ned_deg = start_orientation_ned_degs[2]
    rospy.loginfo('')
    rospy.loginfo("Start Yaw NED degs")
    rospy.loginfo(start_yaw_ned_deg) 
    start_heading_deg=self.current_heading_deg
    rospy.loginfo('')
    rospy.loginfo("Start Heading degs")
    rospy.loginfo(start_heading_deg)   
    ##############################################
    # Condition Body Input Data
    ##############################################
    # Condition Point Input
    input_point_body_m=setpoint_position[0:3]
    rospy.loginfo('')
    rospy.loginfo("Point Input Body Meters")
    rospy.loginfo(" X, Y, Z")
    rospy.loginfo(["%.2f" % input_point_body_m[0],"%.2f" % input_point_body_m[1],"%.2f" % input_point_body_m[2]])
    new_point_body_m=list(input_point_body_m) # No conditioning required
    rospy.loginfo('')
    rospy.loginfo("Point Conditioned Body Meters")
    rospy.loginfo(" X, Y, Z")
    rospy.loginfo(["%.2f" % new_point_body_m[0],"%.2f" % new_point_body_m[1],"%.2f" % new_point_body_m[2]])
    # Condition Orienation Input
    input_yaw_body_deg = setpoint_position[3]
    rospy.loginfo('')
    rospy.loginfo("Yaw Input Body Degrees")
    rospy.loginfo(["%.2f" % input_yaw_body_deg])
    new_yaw_body_deg = input_yaw_body_deg
    # Condition to +-180 deg
    if new_yaw_body_deg > 180:
      new_yaw_body_deg = new_yaw_body_deg - 360
    rospy.loginfo('')
    rospy.loginfo("Yaw Input Conditioned Body Degrees")
    rospy.loginfo(["%.2f" % new_yaw_body_deg])      
    ##############################################
    # Convert Body Data to NED Data
    ##############################################
    # Set new yaw orientation in NED degrees
    offset_ned_m = nepi_nav.convert_point_body2ned(new_point_body_m,start_yaw_ned_deg)
    rospy.loginfo('')
    rospy.loginfo("Point Goal Offsets NED Meters")
    rospy.loginfo(" X, Y, Z")
    rospy.loginfo(["%.2f" % offset_ned_m[0],"%.2f" % offset_ned_m[1],"%.2f" % offset_ned_m[2]])
    new_x_ned_m = start_position_ned_m[0] + offset_ned_m[0]
    new_y_ned_m = start_position_ned_m[1] + offset_ned_m[1]
    new_z_ned_m = start_position_ned_m[2] + offset_ned_m[2]
    new_point_ned_m = [new_x_ned_m,new_y_ned_m,new_z_ned_m]
    rospy.loginfo('')
    rospy.loginfo("Point Goal NED Meters")
    rospy.loginfo(" X, Y, Z")
    rospy.loginfo(["%.2f" % new_point_ned_m[0],"%.2f" % new_point_ned_m[1],"%.2f" % new_point_ned_m[2]])
    new_yaw_ned_deg = nepi_nav.convert_yaw_body2ned(new_yaw_body_deg,start_yaw_ned_deg)
    rospy.loginfo('')
    rospy.loginfo("Yaw Goal NED Degrees")
    rospy.loginfo(["%.2f" % new_yaw_ned_deg])
    ##############################################
    # Convert NED Data to ENU Data
    ##############################################
    # New Point ENU in meters
    new_point_enu_m=Point()
    new_point_enu_m.x = new_point_ned_m[1]
    new_point_enu_m.y = new_point_ned_m[0]
    new_point_enu_m.z = - new_point_ned_m[2]
    rospy.loginfo('')
    rospy.loginfo("Point Goal ENU Meters")
    rospy.loginfo(" X, Y, Z")
    rospy.loginfo(["%.2f" % new_point_enu_m.x,"%.2f" % new_point_enu_m.y,"%.2f" % new_point_enu_m.z])
    new_yaw_enu_deg = nepi_nav.convert_yaw_ned2enu(new_yaw_ned_deg)
    rospy.loginfo('')
    rospy.loginfo("Yaw Goal ENU Degrees")
    rospy.loginfo(["%.2f" % new_yaw_enu_deg])
    ##############################################
    # Create Local ENU Position and Orienation Setpoint Values
    ##############################################
    # New Local Position ENU in meters
    new_point_enu_m=Point()
    new_point_enu_m.x = new_point_enu_m.x
    new_point_enu_m.y = new_point_enu_m.y
    new_point_enu_m.z = new_point_enu_m.z
    rospy.loginfo('')
    rospy.loginfo("Position Goal ENU Meters")
    rospy.loginfo(" X, Y, Z")
    rospy.loginfo(["%.2f" % new_point_enu_m.x,"%.2f" % new_point_enu_m.y,"%.2f" % new_point_enu_m.z])
    # New Local Orienation ENU in meters  
    new_orientation_enu_deg = [start_orientation_ned_degs[0],start_orientation_ned_degs[1],new_yaw_enu_deg]
    rospy.loginfo('')
    rospy.loginfo("Orienation Goal ENU Degrees")
    rospy.loginfo(" Roll, Pitch, Yaw")
    rospy.loginfo(["%.2f" % new_orientation_enu_deg[0],"%.2f" % new_orientation_enu_deg[1],"%.2f" % new_orientation_enu_deg[2]])
    ##############################################
    ## Send Message and Check for Setpoint Success
    ##############################################

    rospy.loginfo('')
    rospy.loginfo("Sending Setpoint Position Local Command at 50 Hz and")
    rospy.loginfo("Waiting for Attitude Setpoint to complete")
    setpoint_position_local_point_reached = False
    setpoint_position_local_yaw_reached = False
    stabilize_timer=0
    point_errors = [] # Initialize running list of errors
    yaw_errors = [] # Initialize running list of errors
    timeout_timer = 0 # Initialize timeout timer
    print_timer_1 = 0
    print_timer_2 = 0
    while setpoint_position_local_point_reached is False or setpoint_position_local_yaw_reached is False and not rospy.is_shutdown():  # Wait for setpoint goal to be set
      if timeout_timer > timeout_sec:
        self.update_error_msg("Setpoint cmd timed out")
        cmd_success = False
        break
      time2sleep = 0.02
      time.sleep(time2sleep) # update setpoint position at 50 Hz
      stabilize_timer=stabilize_timer+time2sleep # Increment rospy.loginfo message timer
      timeout_timer = timeout_timer+time2sleep
      self.setGotoPositionFunction(new_point_enu_m,new_orientation_enu_deg)v
      # Calculate setpoint position ned errors    
      point_ned_error_m = np.array(self.current_position_ned_m) - np.array(new_point_ned_m)
      for ind in range(3):
        if input_point_body_m == -999: # Ignore error check if set to current
          point_ned_error_m[ind] = 0
      max_point_ned_error_m = np.max(np.abs(point_ned_error_m))
      # Calculate setpoint yaw ned error
      if input_yaw_body_deg == -999: # Ignore error check if set to current
        setpoint_position_local_yaw_reached = True
        max_yaw_ned_error_deg = 0
      else:
        cur_yaw_ned_deg = self.current_orientation_ned_degs[2]
        yaw_ned_error_deg =  cur_yaw_ned_deg - new_yaw_ned_deg
        max_yaw_ned_error_deg = abs(yaw_ned_error_deg)
      # Check for setpoint position local point goal
      if  setpoint_position_local_point_reached is False:
        if stabilize_timer > GOTO_STABILIZED_SEC:
          max_point_errors = max(point_errors) # Get max from error window
          point_errors = [max_point_ned_error_m] # reset running list of errors
        print_timer_1 = print_timer_1 + time2sleep
        if print_timer_1 > 1:
          print_timer_1 = 0
          rospy.loginfo("Goto Position Position Updates")
          # rospy.loginfo some information every second
          rospy.loginfo('')
          rospy.loginfo("Current Position NED Meters")
          rospy.loginfo(" X, Y, Z")
          rospy.loginfo(["%.2f" % self.current_position_ned_m[0],"%.2f" % self.current_position_ned_m[1],"%.2f" % self.current_position_ned_m[2]])
          rospy.loginfo("Current Goal NED Meters")
          rospy.loginfo(" X, Y, Z")
          rospy.loginfo(["%.2f" % new_point_ned_m[0],"%.2f" % new_point_ned_m[1],"%.2f" % new_point_ned_m[2]])
          rospy.loginfo("Current Errors Meters")
          rospy.loginfo(" X, Y, Z")
          rospy.loginfo(["%.2f" % point_ned_error_m[0],"%.2f" % point_ned_error_m[1],"%.2f" % point_ned_error_m[2]])
          rospy.loginfo("Max Error from Stabilized Check Window Meters")
          rospy.loginfo(["%.2f" % max_point_errors])
          if max_point_errors < self.rbx_status.error_bounds.max_distance_error_m:
            rospy.loginfo('')
            rospy.loginfo("Position Setpoint Reached")
            setpoint_position_local_point_reached = True
        else:
          point_errors.append(max_point_ned_error_m) # append last
      # Check for setpoint position yaw point goal
      if  setpoint_position_local_yaw_reached is False:
        if stabilize_timer > self.rbx_status.error_bounds.max_stabilize_time_s:
          max_yaw_errors = max(yaw_errors) # Get max from error window
          yaw_errors = [max_yaw_ned_error_deg] # reset running list of errors
        print_timer_2 = print_timer_2 + time2sleep
        if print_timer_2 > 1:
          print_timer_2 = 0
          rospy.loginfo("")
          rospy.loginfo("Goto Position Yaw Updates")
          # rospy.loginfo some information every second
          rospy.loginfo('')
          rospy.loginfo("Current Yaw NED Degrees")
          rospy.loginfo(self.current_orientation_ned_degs[2])
          rospy.loginfo("Current Goal NED Degrees")
          rospy.loginfo(new_yaw_ned_deg)
          rospy.loginfo("Current Error Degree")
          rospy.loginfo(max_yaw_ned_error_deg)
          rospy.loginfo("Max Error from Stabilized Check Window Meters")
          rospy.loginfo(["%.2f" % max_yaw_errors])
          if max_yaw_errors < self.rbx_status.error_bounds.max_rotation_error_deg:
            rospy.loginfo('')
            rospy.loginfo("Yaw Setpoint Reached")
            setpoint_position_local_yaw_reached = True
        else:
          yaw_errors.append(max_yaw_ned_error_deg) # append last
      # Reset rospy.loginfo timer if past
      if stabilize_timer > self.rbx_status.error_bounds.max_stabilize_time_s:
        stabilize_timer=0 # Reset rospy.loginfo timer
      self.update_current_errors(  [point_ned_error_m[0],point_ned_error_m[1],point_ned_error_m[2],0,0,0,max_yaw_ned_error_deg] )
    if cmd_success:
      rospy.loginfo("************************")
      rospy.loginfo("Setpoint Reached")
    self.update_current_errors( [0,0,0,0,0,0,0] )
    self.update_prev_errors(  [point_ned_error_m[0],point_ned_error_m[1],point_ned_error_m[2],0,0,0,max_yaw_ned_error_deg] )
    return cmd_success



  ### Function to set and check setpoint location global geopoint and yaw command
  ###################################################
  # Input is [LAT, LONG, ALT_WGS84, YAW_NED_DEGREES]
  # Converted to AMSL Altitude and ENU Yaw berore sending
  # Altitude is specified as meters above the WGS-84 and converted to AMSL before sending
  # Yaw is specified in NED frame degrees 0-360 or +-180 
  #####################################################
  def setpoint_location_global_wgs84(self,setpoint_location,timeout_sec=CMD_TIMEOUT_SEC):
    # setpoint_location is [LAT, LONG, ALT_WGS84, YEW_NED_DEGREES 0-360 or +-180]
    # Use value -999 to use current value
    cmd_success = True
    self.update_current_errors( [0,0,0,0,0,0,0] )
    rospy.loginfo('')
    rospy.loginfo("Starting Setpoint Location Global Create-Send-Check Process")
    ##############################################
    # Capture Current NavPose Data
    ##############################################
    start_geopoint_wgs84 = list(self.current_location_wgs84_geo)  
    rospy.loginfo('')
    rospy.loginfo("Start Location WSG84 geopoint")
    rospy.loginfo(" Lat, Long, Alt")
    rospy.loginfo(["%.6f" % start_geopoint_wgs84[0],"%.6f" % start_geopoint_wgs84[1],"%.2f" % start_geopoint_wgs84[2]])
    start_orientation_ned_degs=list(self.current_orientation_ned_degs)
    rospy.loginfo('')
    rospy.loginfo("Start Orientation NED degs")
    rospy.loginfo(" Roll, Pitch, Yaw")
    rospy.loginfo(["%.6f" % start_orientation_ned_degs[0],"%.6f" % start_orientation_ned_degs[1],"%.2f" % start_orientation_ned_degs[2]])
    rospy.loginfo('')
    start_yaw_ned_deg = start_orientation_ned_degs[2]
    if start_yaw_ned_deg < 0:
      start_yaw_ned_deg = start_yaw_ned_deg + 360
    rospy.loginfo('')
    rospy.loginfo("Start Yaw NED degs 0-360")
    rospy.loginfo(start_yaw_ned_deg) 
    start_heading_deg=self.current_heading_deg
    rospy.loginfo('')
    rospy.loginfo("Start Heading degs")
    rospy.loginfo(start_heading_deg)
    start_geoid_height_m = self.current_geoid_height_m
    ##############################################
    # Condition NED Input Data
    ##############################################
    # Condition Location Input
    input_geopoint_wgs84 = list(setpoint_location[0:3])
    rospy.loginfo('')
    rospy.loginfo("Location Input Global Geo")
    rospy.loginfo(" Lat, Long, Alt_WGS84")
    rospy.loginfo(["%.8f" % input_geopoint_wgs84[0],"%.8f" % input_geopoint_wgs84[1],"%.2f" % input_geopoint_wgs84[2]])
    new_geopoint_wgs84=list(start_geopoint_wgs84) # Initialize with start
    for ind in range(3): # Overwrite current with new if set and valid
      if input_geopoint_wgs84[ind] != -999:
        new_geopoint_wgs84[ind]=input_geopoint_wgs84[ind]
    rospy.loginfo('')
    rospy.loginfo("Location Input Conditioned Global Geo")
    rospy.loginfo(" Lat, Long, Alt_WGS84")
    rospy.loginfo(["%.8f" % new_geopoint_wgs84[0],"%.8f" % new_geopoint_wgs84[1],"%.2f" % new_geopoint_wgs84[2]])
    # Condition Yaw Input
    input_yaw_ned_deg = setpoint_location[3]
    rospy.loginfo('')
    rospy.loginfo("Yaw Input NED Degrees")
    rospy.loginfo(["%.2f" % input_yaw_ned_deg])
    new_yaw_ned_deg = start_yaw_ned_deg # Initialize to current
    if input_yaw_ned_deg != -999: # Replace if not -999
      new_yaw_ned_deg = input_yaw_ned_deg
    # Condition to 0-360 degs
    if new_yaw_ned_deg < 0:
      new_yaw_ned_deg = new_yaw_ned_deg + 360
    rospy.loginfo('')
    rospy.loginfo("Yaw Input Conditioned NED Degrees 0-360")
    rospy.loginfo(["%.2f" % new_yaw_ned_deg])      
    ##############################################
    # Create Global AMSL Location and NED Orienation Setpoint Values
    ##############################################
    # New Global location ENU in meters
    new_geopoint_amsl=GeoPoint()
    new_geopoint_amsl.latitude = new_geopoint_wgs84[0]
    new_geopoint_amsl.longitude = new_geopoint_wgs84[1]
    new_geopoint_amsl.altitude = new_geopoint_wgs84[2] + start_geoid_height_m
    rospy.loginfo('')
    rospy.loginfo("Location Goal AMSL Meters")
    rospy.loginfo(" Lat, Long, Alt_AMSL")
    rospy.loginfo(["%.8f" % new_geopoint_amsl.latitude,"%.8f" % new_geopoint_amsl.longitude,"%.2f" % new_geopoint_amsl.altitude])
    # New Local Orienation NED in degs  
    new_orientation_ned_deg = [start_orientation_ned_degs[0],start_orientation_ned_degs[1],new_yaw_ned_deg]
    rospy.loginfo('')
    rospy.loginfo("Orienation Goal NED Degrees")
    rospy.loginfo(" Roll, Pitch, Yaw")
    rospy.loginfo(["%.2f" % new_orientation_ned_deg[0],"%.2f" % new_orientation_ned_deg[1],"%.2f" % new_orientation_ned_deg[2]])
     ##############################################
    ## Send Message and Check for Setpoint Success
    ##############################################
 
    rospy.loginfo("Sending MAVLINK Setpoint Position Local Command at 50 Hz and")
    rospy.loginfo(" checking for Setpoint Reached")
    setpoint_location_global_geopoint_reached = False
    setpoint_location_global_yaw_reached = False 
    rospy.loginfo('')
    rospy.loginfo("Waiting for Position Local Setpoint to complete")
    stabilize_timer=0
    geopoint_errors = [] # Initialize running list of errors
    yaw_errors = [] # Initialize running list of errors
    timeout_timer = 0 # Initialize timeout timer
    print_timer_1 = 0
    print_timer_2 = 0
    while setpoint_location_global_geopoint_reached is False or setpoint_location_global_yaw_reached is False and not rospy.is_shutdown(): # Wait for setpoint goal to be set
      if timeout_timer > timeout_sec:
        self.update_error_msg("Setpoint cmd timed out")
        cmd_success = False
        break
      time2sleep = 0.02
      time.sleep(time2sleep) # update setpoint position at 50 Hz
      stabilize_timer=stabilize_timer+time2sleep # Increment self.update_error_msg message timer
      timeout_timer = timeout_timer+time2sleep
      sefl.setGotoLocationFunction(new_geopoint_amsl,new_orientation_ned_deg)
      
      # Calculate setpoint position and yaw errors
      geopoint_errors_geo = np.array(self.current_location_wgs84_geo) - np.array(new_geopoint_wgs84)
      geopoint_errors_m = [geopoint_errors_geo[0]*111139,geopoint_errors_geo[1]*111139,geopoint_errors_geo[2]]
      for ind in range(3):  # Ignore error check if set to current
        if input_geopoint_wgs84[ind] == -999.0:
          geopoint_errors_m[ind] = 0
      max_geopoint_error_m = np.max(np.abs(geopoint_errors_m))
      if input_yaw_ned_deg == -999: # Ignore error check if set to current
        setpoint_location_global_yaw_reached = True
        max_yaw_ned_error_deg = 0
      else:
        cur_yaw_ned_deg = self.current_orientation_ned_degs[2]
        if cur_yaw_ned_deg < 0:
          cur_yaw_ned_deg = cur_yaw_ned_deg + 360
        yaw_ned_error_deg =  cur_yaw_ned_deg - new_yaw_ned_deg
        max_yaw_ned_error_deg = abs(yaw_ned_error_deg)
      # Check for setpoint position global goal
      if  setpoint_location_global_geopoint_reached is False:
        if stabilize_timer > self.rbx_status.error_bounds.max_stabilize_time_s:
          max_geopoint_errors = max(geopoint_errors) # Get max from error window
          geopoint_errors = [max_geopoint_error_m] # reset running list of errors
        print_timer_1 = print_timer_1 + time2sleep
        if print_timer_1 > 1:
          print_timer_1 = 0
          rospy.loginfo("")
          rospy.loginfo("Goto Location Location Updates")
          rospy.loginfo(self.rbx_status.errors_current)
          # rospy.loginfo some information every second
          rospy.loginfo('')
          rospy.loginfo("Current Location WGS84")
          rospy.loginfo(" Lat, Long, Alt_WGS84")
          rospy.loginfo(["%.7f" % self.current_location_wgs84_geo[0],"%.7f" % self.current_location_wgs84_geo[1],"%.2f" % self.current_location_wgs84_geo[2]])
          rospy.loginfo("Current Goal WGS84")
          rospy.loginfo(" Lat, Long, Alt_WGS84")
          rospy.loginfo(["%.7f" % new_geopoint_wgs84[0],"%.7f" % new_geopoint_wgs84[1],"%.2f" % new_geopoint_wgs84[2]])
          rospy.loginfo("Current Errors Meters")
          rospy.loginfo(" Lat, Long, Alt")
          rospy.loginfo(["%.2f" % geopoint_errors_m[0],"%.2f" % geopoint_errors_m[1],"%.2f" % geopoint_errors_m[2]])
          rospy.loginfo("Max Error from Stabilized Check Window Meters")
          rospy.loginfo(["%.2f" % max_geopoint_errors])
          if max_geopoint_errors < self.rbx_status.error_bounds.max_distance_error_m:
            rospy.loginfo('')
            rospy.loginfo("Location Setpoint Reached")
            setpoint_location_global_geopoint_reached = True
        else:
          geopoint_errors.append(max_geopoint_error_m) # append last
      # Check for setpoint position yaw goal
      if  setpoint_location_global_yaw_reached is False:
        if stabilize_timer > self.rbx_status.error_bounds.max_stabilize_time_s:
          max_yaw_errors = max(yaw_errors) # Get max from error window
          yaw_errors = [max_yaw_ned_error_deg] # reset running list of errors
        print_timer_2 = print_timer_2 + time2sleep
        if print_timer_2 > 1:
          print_timer_2 = 0
          rospy.loginfo("")
          rospy.loginfo("Goto Location Yaw Updates")
          # rospy.loginfo some information every second
          rospy.loginfo('')
          rospy.loginfo("Current Yaw NED Degrees")
          rospy.loginfo(cur_yaw_ned_deg)
          rospy.loginfo("Current Goal NED Degrees")
          rospy.loginfo(new_yaw_ned_deg)
          rospy.loginfo("Current Error Degree")
          rospy.loginfo(max_yaw_ned_error_deg)
          rospy.loginfo("Max Error from Stabilized Check Window Degs")
          rospy.loginfo(["%.2f" % max_yaw_errors])
          if max_yaw_errors < self.rbx_status.error_bounds.max_rotation_error_deg:
            rospy.loginfo('')
            rospy.loginfo("Yaw Setpoint Reached")
            setpoint_location_global_yaw_reached = True
        else:
          yaw_errors.append(max_yaw_ned_error_deg) # append last
      # Reset rospy.loginfo timer if past
      if stabilize_timer > 1:
        stabilize_timer=0 # Reset rospy.loginfo timer
      self.update_current_errors( [geopoint_errors_m[0],geopoint_errors_m[1],geopoint_errors_m[2],0,0,0,max_yaw_ned_error_deg] )
    if cmd_success:
      rospy.loginfo("************************")
      rospy.loginfo("Setpoint Reached")
    self.update_current_errors( [0,0,0,0,0,0,0] )
    self.update_prev_errors( [geopoint_errors_m[0],geopoint_errors_m[1],geopoint_errors_m[2],0,0,0,max_yaw_ned_error_deg] )
    return cmd_success

  #######################
  # Class Utility Functions
  
  ### Function for updating current goto error values
  def update_current_errors(self,error_list):
    if len(error_list) == 7:
      self.rbx_status.errors_current.x_m = error_list[0]
      self.rbx_status.errors_current.y_m = error_list[1]
      self.rbx_status.errors_current.z_m = error_list[2]
      self.rbx_status.errors_current.heading_deg = error_list[3]
      self.rbx_status.errors_current.roll_deg = error_list[4]
      self.rbx_status.errors_current.pitch_deg = error_list[5]
      self.rbx_status.errors_current.yaw_deg = error_list[6]
    else:
      rospy.loginfo("Skipping current error update. Error list to short")

  ### Function for updating last goto error values
  def update_prev_errors(self,error_list):
    if len(error_list) == 7:
      self.rbx_status.errors_prev.x_m = error_list[0]
      self.rbx_status.errors_prev.y_m = error_list[1]
      self.rbx_status.errors_prev.z_m = error_list[2]
      self.rbx_status.errors_prev.heading_deg = error_list[3]
      self.rbx_status.errors_prev.roll_deg = error_list[4]
      self.rbx_status.errors_prev.pitch_deg = error_list[5]
      self.rbx_status.errors_prev.yaw_deg = error_list[6]
    else:
      rospy.loginfo("Skipping current error update. Error list to short")

  def update_error_msg(self,error_msg):
    rospy.loginfo(error_msg)
    self.status_msg.last_error_message = error_msg

  def get_motor_controls_status_msg(self,motor_controls):
    mcs = []
    for i in range(len(motor_controls)):
      mcs.append([str(i),str(motor_controls[i])])
    return str(mcs)


  #************************************************************

# Define saving functions for saving callbacks



    def save_img2file(self,data_product,cv2_img,ros_timestamp):
        if self.save_data_if is not None:
            saving_is_enabled = self.save_data_if.data_product_saving_enabled(data_product)
            snapshot_enabled = self.save_data_if.data_product_snapshot_enabled(data_product)
            # Save data if enabled
            if saving_is_enabled or snapshot_enabled:
                eval("self." + data_product + "_lock.acquire()")
                if cv2_img is not None:
                    device_name = rospy.get_param('~rbx/device_name', self.init_device_name)
                    if (self.save_data_if.data_product_should_save(data_product) or snapshot_enabled):
                        full_path_filename = self.save_data_if.get_full_path_filename(nepi_ros.get_datetime_str_from_stamp(ros_timestamp), 
                                                                                                device_name + "-" + data_product, 'png')
                        if os.path.isfile(full_path_filename) is False:
                            cv2.imwrite(full_path_filename, cv2_img)
                            self.save_data_if.data_product_snapshot_reset(data_product)
                eval("self." + data_product + "_lock.release()")


    def saveImgThread(self,timer):
        data_product = 'color_2d_image'
        eval("self.save_img2file(data_product,self." + data_product + ",self." + data_product + "_timestamp)")
       


    
