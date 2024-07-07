#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# Sample NEPI Driver Script. 
# NEPI RBX Driver for Ardupilot Autopilot Systems

### Set the namespace before importing rospy
import os
import rospy
import time
import numpy as np
import math
import tf
import random
import sys
import cv2
from nepi_edge_sdk_base import nepi_ros 
from nepi_edge_sdk_base import nepi_nav

from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandHome


# ROS namespace setup
ROS_NAMESPACE = nepi_ros.get_base_namespace()



#########################################
# DRIVER SETTINGS
#########################################

##############################

##############################


##############################
# ARDUPILOT Settings
TAKEOFF_MIN_PITCH_DEG = 10
TAKEOFF_ALT_M = 1

STATUS_UPDATE_RATE_HZ = 20

#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()

# RBX node name
NEPI_RBX_NODENAME = "ardupilot"

#########################################
# Node Class
#########################################

class ardupilot_rbx_node(object):

  DEFAULT_NODE_NAME = "ardupilot" # connection port added once discovered

  CAP_SETTINGS = [["Float","takeoff_height_m","0.0","10000.0"]]

  FACTORY_SETTINGS_OVERRIDE = dict( takeoff_height_m = "5" )

  settings_dict = FACTORY_SETTINGS_OVERRIDE 

  axis_controls = AxisControls()
  axis_controls.x = True
  axis_controls.y = True
  axis_controls.z = True
  axis_controls.roll = True
  axis_controls.pitch = True
  axis_controls.yaw = True



  # Create shared class variables and thread locks 
  
  device_info_dict = dict(node_name = "",
                          sensor_name = "",
                          identifier = "",
                          serial_number = "",
                          hw_version = "",
                          sw_version = "")

  state_ind = -999
  mode_ind = -999
  battery_percent = -999

  # RBX State and Mode Dictionaries
  RBX_NAVPOSE_HAS_GPS = True
  RBX_NAVPOSE_HAS_ORIENTATION = True
  RBX_NAVPOSE_HAS_HEADING = True

  RBX_STATES = ["DISARM","ARM"]
  RBX_MODES = ["MANUAL","STABILIZE","LAND","RTL","LOITER","GUIDED","RESUME"]
  RBX_ACTIONS = ["LAUNCH","TAKEOFF"]

  RBX_STATE_FUNCTIONS = ["disarm","arm"]
  RBX_MODE_FUNCTIONS = ["manual","stabilize","land","rtl","loiter","guided","resume"]
  RBX_ACTION_FUNCTIONS = ["launch","takeoff"]





  #######################
  ### Node Initialization
  def __init__(self):
    rospy.loginfo("Starting Initialization Processes")

    # Launch the NEPI ROS RBX node
    node_name = self.DEFAULT_NODE_NAME
    if port_id is not None:
      node_name = node_name + '_' + port_id
    rospy.loginfo("")
    rospy.loginfo("********************")
    rospy.loginfo("Starting " + node_name)
    rospy.loginfo("********************")
    rospy.loginfo("")
    rospy.init_node(node_name) # Node name could be overridden via remapping
    self.node_name = rospy.get_name().split('/')[-1]
    rospy.loginfo(self.node_name + ": ... Connected!")


    mavlink_node_name = rospy.get_param('~mavlink_node_name', default='mavlink')
    node_id = mavlink_node_name.replace("mavlink","")

    ###################################################
    # MAVLINK Namespace
    # Find Mavlink NameSpace
    node_string = mavlink_node_name
    rospy.loginfo("Waiting for node that includes string: " + node_string)
    node_name = nepi_ros.wait_for_node(node_string)
    MAVLINK_NAMESPACE = (node_name + '/')
    rospy.loginfo("Found mavlink namespace: " + MAVLINK_NAMESPACE)
    # MAVLINK Subscriber Topics
    MAVLINK_STATE_TOPIC = MAVLINK_NAMESPACE + "state"
    MAVLINK_BATTERY_TOPIC = MAVLINK_NAMESPACE + "battery"
    # MAVLINK Required Services
    MAVLINK_SET_HOME_SERVICE = MAVLINK_NAMESPACE + "cmd/set_home"
    MAVLINK_SET_MODE_SERVICE = MAVLINK_NAMESPACE + "set_mode"
    MAVLINK_ARMING_SERVICE = MAVLINK_NAMESPACE + "cmd/arming"
    MAVLINK_TAKEOFF_SERVICE = MAVLINK_NAMESPACE + "cmd/takeoff"
    # MAVLINK NavPose Source Topics
    MAVLINK_SOURCE_GPS_TOPIC = MAVLINK_NAMESPACE + "global_position/global"
    MAVLINK_SOURCE_ODOM_TOPIC = MAVLINK_NAMESPACE + "global_position/local"
    MAVLINK_SOURCE_HEADING_TOPIC = MAVLINK_NAMESPACE + "global_position/compass_hdg"
    # MAVLINK Setpoint Control Topics
    MAVLINK_SETPOINT_ATTITUDE_TOPIC = MAVLINK_NAMESPACE + "setpoint_raw/attitude"
    MAVLINK_SETPOINT_POSITION_LOCAL_TOPIC = MAVLINK_NAMESPACE + "setpoint_position/local"
    MAVLINK_SETPOINT_LOCATION_GLOBAL_TOPIC = MAVLINK_NAMESPACE + "setpoint_position/global"

    # NEPI RBX Driver NavPose Publish Topics
    NEPI_RBX_NAVPOSE_GPS_TOPIC = NEPI_RBX_NAMESPACE + "gps_fix"
    NEPI_RBX_NAVPOSE_ODOM_TOPIC = NEPI_RBX_NAMESPACE + "odom"
    NEPI_RBX_NAVPOSE_HEADING_TOPIC = NEPI_RBX_NAMESPACE + "heading"

    ## Start Class Subscribers
    # Wait for MAVLink State topic to publish then subscribe
    rospy.loginfo("Waiting for topic: " + MAVLINK_STATE_TOPIC)
    nepi_ros.wait_for_topic(MAVLINK_STATE_TOPIC)
    rospy.loginfo("Starting state scubscriber callback")
    rospy.Subscriber(MAVLINK_STATE_TOPIC, State, self.get_state_callback)
    while self.rbx_status.state is None and not rospy.is_shutdown():
    rospy.loginfo("Waiting for rbx state status to set")
    time.sleep(0.1)
    while self.rbx_status.mode is None and not rospy.is_shutdown():
    rospy.loginfo("Waiting for rbx mode status to set")
    time.sleep(0.1)
    rospy.loginfo("Starting State: " + self.RBX_STATES[self.rbx_status.state])
    rospy.loginfo("Starting Mode: " + self.RBX_MODES[self.rbx_status.mode])
    self.rbx_state_start = self.rbx_status.state
    self.rbx_mode_start = self.rbx_status.mode
    self.rbx_state_last = self.rbx_status.state
    self.rbx_mode_last = self.rbx_status.mode


    ## Define RBX NavPose Publishers
    NEPI_RBX_NAMESPACE = NEPI_BASE_NAMESPACE + self.node_name + "/rbx/"
    self.rbx_navpose_gps_pub = rospy.Publisher(NEPI_RBX_NAVPOSE_GPS_TOPIC, NavSatFix, queue_size=1)
    self.rbx_navpose_odom_pub = rospy.Publisher(NEPI_RBX_NAVPOSE_ODOM_TOPIC, Odometry, queue_size=1)
    self.rbx_navpose_heading_pub = rospy.Publisher(NEPI_RBX_NAVPOSE_HEADING_TOPIC, Float64, queue_size=1)


    ## Define Mavlink Services Calls
    self.get_navpose_service = rospy.ServiceProxy(nepi_nav_SERVICE_NAME, NavPoseQuery)
    self.set_home_client = rospy.ServiceProxy(MAVLINK_SET_HOME_SERVICE, CommandHome)
    self.mode_client = rospy.ServiceProxy(MAVLINK_SET_MODE_SERVICE, SetMode)
    self.arming_client = rospy.ServiceProxy(MAVLINK_ARMING_SERVICE, CommandBool)
    self.takeoff_client = rospy.ServiceProxy(MAVLINK_TAKEOFF_SERVICE, CommandTOL)


    # Subscribe to MAVLink topics
    rospy.Subscriber(MAVLINK_BATTERY_TOPIC, BatteryState, self.get_mavlink_battery_callback)
    rospy.Subscriber(MAVLINK_SOURCE_GPS_TOPIC, NavSatFix, self.gps_topic_callback)
    rospy.Subscriber(MAVLINK_SOURCE_ODOM_TOPIC, Odometry, self.odom_topic_callback)
    rospy.Subscriber(MAVLINK_SOURCE_HEADING_TOPIC, Float64, self.heading_topic_callback)
    
    ## Define Mavlink Publishers
    self.setpoint_location_global_pub = rospy.Publisher(MAVLINK_SETPOINT_LOCATION_GLOBAL_TOPIC, GeoPoseStamped, queue_size=1)
    self.setpoint_attitude_pub = rospy.Publisher(MAVLINK_SETPOINT_ATTITUDE_TOPIC, AttitudeTarget, queue_size=1)
    self.setpoint_position_local_pub = rospy.Publisher(MAVLINK_SETPOINT_POSITION_LOCAL_TOPIC, PoseStamped, queue_size=1)


    # Initialize settings
    self.cap_settings = self.getCapSettings()
    rospy.loginfo("CAPS SETTINGS")
    #for setting in self.cap_settings:
        #rospy.loginfo(setting)
    self.factory_settings = self.getFactorySettings()
    rospy.loginfo("FACTORY SETTINGS")
    #for setting in self.factory_settings:
        #rospy.loginfo(setting)
          

    # Launch the IDX interface --  this takes care of initializing all the camera settings from config. file
    rospy.loginfo(self.node_name + ": Launching NEPI IDX (ROS) interface...")
    self.device_info_dict["node_name"] = self.node_name
    self.device_info_dict["device_name"] = self.node_name
    if self.node_name.find("_") != -1:
        split_name = self.node_name.rsplit('_', 1)
        self.device_info_dict["identifier"] = split_name[1] + split_name[1]
    else:
        self.device_info_dict["identifier"] = ""
    
    self.device_info_dict["serial_number"] = ""
    self.device_info_dict["hw_version"] = ""
    self.device_info_dict["sw_version"] = ""

    self.idx_if = ROSIDXSensorIF(device_info = self.device_info_dict,
                                  capSettings = self.cap_settings,
                                  factorySettings = self.factory_settings,
                                  settingUpdateFunction=self.settingUpdateFunction,
                                  getSettingsFunction=self.getSettings,
                                  axisControls = self.axis_controls,
                                  getBatteryPercentFunction = self.getBatteryPercent,
                                  states = self.RBX_STATES,
                                  getStateIndFunction = self.getStateInd,
                                  setStateIndFunction = self.setStateInd,
                                  modes = self.RBX_MODES,
                                  getModeIndFunction = self.getModeInd,
                                  setModeIndFunction = self.setModeInd,
                                  actions = self.RBX_ACTIONS, 
                                  setActionIndFunction = self.setActionInd,
                                  manualControlsReadyFunction = self.manualControlsReady,
                                  autonomousControlsReadyFunction = self.autonomousControlsReady,
                                  setHomeCurrentFunction = self.setHomeCurrent,
                                  goHomeFunction = self.goHomeFunction, 
                                  goStopFunction = self.goStopFunction, 
                                  gotoPoseFunction = self.gotoPoseFunction,
                                  gotoPositionFunction = self.gotoPositionFunction, 
                                  gotoLocationFunction = self.gotoLocationFunction,
                                  motorControlFunction = self.set_motor_control,
                                  gpsTopic = NEPI_RBX_NAVPOSE_GPS_TOPIC,
                                  odomTopic = NEPI_RBX_NAVPOSE_ODOM_TOPIC,
                                  headingTopic = NEPI_RBX_NAVPOSE_HEADING_TOPIC
    )


    rospy.loginfo(self.node_name + ": ... IDX interface running")

    # Update available IDX callbacks based on capabilities that the driver reports
    self.logDeviceInfo()

    # Configure pointcloud processing Verbosity
    pc_verbosity = nepi_pc.set_verbosity_level("Error")
    rospy.loginfo(pc_verbosity)

    # Now that all camera start-up stuff is processed, we can update the camera from the parameters that have been established
    self.idx_if.updateFromParamServer()

    # Now start the node
    rospy.spin()


   #**********************
    # Setting functions
    def getCapSettings(self):
      return self.CAP_SETTINGS

    def getFactorySettings(self):
      settings = self.getSettings()
      #Apply factory setting overides
      for setting in settings:
        if setting[1] in self.FACTORY_SETTINGS_OVERRIDES:
              setting[2] = self.FACTORY_SETTINGS_OVERRIDES[setting[1]]
              settings = nepi_ros.update_setting_in_settings(setting,settings)
      return settings


    def getSettings(self):
      settings = []
      settings_dict = self.settings_dict # Temp for testing
      if settings_dict is not None:
        for cap_setting in self.cap_settings:
          setting_type = cap_setting[0]
          setting_name = cap_setting[1]
          if setting_name in settings_dict.keys():
            setting_value = settings_dict[setting_name]
            setting = [setting_type,setting_name,str(setting_value)]
            settings.append(setting)
      return settings

    def settingUpdateFunction(self,setting):
      success = False
      setting_str = str(setting)
      if len(setting) == 3:
        setting_type = setting[0]
        setting_name = setting[1]
        [s_name, s_type, data] = nepi_ros.get_data_from_setting(setting)
        #rospy.loginfo(type(data))
        if data is not None:
          setting_data = data
          settings_dict = self.settings_dict
          if setting_name in settings_dict.keys():
            self.settings_dict[setting_name] = setting[2]
            success = True
            msg = ( self.node_name  + " UPDATED SETTINGS " + setting_str)
          else:
            msg = (self.node_name  + " Setting name" + setting_str + " is not supported")                   
        else:
          msg = (self.node_name  + " Setting data" + setting_str + " is None")
      else:
        msg = (self.node_name  + " Setting " + setting_str + " not correct length")
      return success, msg

  ##########################
  # RBX Interface Functions

  def getStateInd(self):
    return self.state_ind

  def setStateInd(self,state_ind):
    set_state_function = globals()[self.RBX_STATE_FUNCTIONS[state_ind]]
    success = set_state_function(self)
    return success

  def getModeInd(self):
    return self.mode_ind

  def setModeInd(self,mode_ind):
    set_mode_function = globals()[self.RBX_MODE_FUNCTIONS[mode_ind]]
    success = set_mode_function(self)
    return success
    
  def getBatteryPercent(self):
    return self.battery_percent

  def setHomeCurrent(self):
    self.sethome_current()

  def setMotorControlRatio(self,motor_ind,speed_ratio):
    pass

  def getMotorControlRatios(self):
    return []

  def setActionInd(self,action_ind):
    set_action_function = globals()[self.RBX_ACTION_FUNCTIONS[action_ind]]
    success = set_action_function(self,self.rbx_status.cmd_timeout)
    return success

  def setGoHome(self):
    self.rtl(self)
    return True

  def setGoStop(self):
    self.loiter()
    return True

  def setGotoPose(self,attitude_enu_degs):
    # Create Setpoint Attitude Message
    attitude_enu_quat = nepi_nav.convert_rpy2quat(attitude_enu_degs)
    orientation_enu_quat = Quaternion()
    orientation_enu_quat.x = attitude_enu_quat[0]
    orientation_enu_quat.y = attitude_enu_quat[1]
    orientation_enu_quat.z = attitude_enu_quat[2]
    orientation_enu_quat.w = attitude_enu_quat[3]
    # Set other setpoint attitude message values
    body_rate = Vector3()
    body_rate.x = 0
    body_rate.y = 0
    body_rate.z = 0
    type_mask = 1|2|4
    thrust_ratio = 0
    rospy.loginfo('')
    rospy.loginfo("Creating Message")
    attitude_target_msg = AttitudeTarget()
    attitude_target_msg.orientation = orientation_enu_quat
    attitude_target_msg.body_rate = body_rate
    attitude_target_msg.type_mask = type_mask
    attitude_target_msg.thrust = thrust_ratio
    rospy.loginfo('')
    rospy.loginfo("Setpoint Goal Attitude ENU Message")
    rospy.loginfo(attitude_target_msg)
    ## Send Setpoint Message
    self.setpoint_attitude_pub.publish(attitude_target_msg) # Publish Setpoint

  def setGotoPosition(self,point_enu_m,orientation_enu_deg):
    # Create PoseStamped Setpoint Local ENU Message
    orientation_enu_q = nepi_nav.convert_rpy2quat(orientation_enu_deg)
    orientation_enu_quat = Quaternion()
    orientation_enu_quat.x = orientation_enu_q[0]
    orientation_enu_quat.y = orientation_enu_q[1]
    orientation_enu_quat.z = orientation_enu_q[2]
    orientation_enu_quat.w = orientation_enu_q[3]
    pose_enu=Pose()
    pose_enu.position = point_enu_m
    pose_enu.orientation = orientation_enu_quat
    position_local_target_msg = PoseStamped()
    position_local_target_msg.pose = pose_enu
    rospy.loginfo('')
    rospy.loginfo("Setpoint Goal Position Local Message")
    rospy.loginfo(position_local_target_msg)
    ## Send Message and Check for Setpoint Success
    self.setpoint_position_local_pub.publish(position_local_target_msg) # Publish Setpoint

  def setGotoLocation(self,geopoint_amsl,orientation_ned_deg):
    # Create GeoPose Setpoint Global AMSL and Yaw NED Message
    orientation_ned_q = nepi_nav.convert_rpy2quat(orientation_ned_deg)
    orientation_ned_quat = Quaternion()
    orientation_ned_quat.x = orientation_ned_q[0]
    orientation_ned_quat.y = orientation_ned_q[1]
    orientation_ned_quat.z = orientation_ned_q[2]
    orientation_ned_quat.w = orientation_ned_q[3]
    geopose_enu=GeoPose()
    geopose_enu.position = geopoint_amsl
    geopose_enu.orientation = orientation_ned_quat
    location_global_target_msg = GeoPoseStamped()
    location_global_target_msg.pose = geopose_enu
    rospy.loginfo('')
    rospy.loginfo("Setpoint Location Goal Message")
    rospy.loginfo(location_global_target_msg)
    ##############################################
    ## Send Message and Check for Setpoint Success
    ##############################################
    self.setpoint_location_global_pub.publish(location_global_target_msg) # Publish Setpoint

  ##########################
  # Control Ready Check Funcitons

  def manualControlsReady(self):
    ready = False
    if self.mode_ind < len(self.RBX_MODES):
      if self.RBX_MODES[self.mode_ind] = "MANUAL":
        ready = True
    return ready

  def autonomousControlsReady(self):
    ready = False
    if self.mode_ind < len(self.RBX_MODES):
      if self.RBX_MODES[self.mode_ind] = "GUIDED":
        ready = True
    return ready

  ##############################
  # RBX NavPose Topic Publishers
  ### Callback to publish RBX navpose data
  
  def gps_topic_callback(self,navsatfix_msg):
      #Fix Mavros Altitude Error
      altitude_wgs84 = navsatfix_msg.altitude - self.current_geoid_height_m
      navsatfix_msg.altitude = altitude_wgs84 
      if not rospy.is_shutdown():
      self.rbx_navpose_gps_pub.publish(navsatfix_msg)
      
  ### Callback to publish RBX odom topic
  def odom_topic_callback(self,odom_msg):
      if not rospy.is_shutdown():
      self.rbx_navpose_odom_pub.publish(odom_msg)

  ### Callback to publish RBX heading topic
  def heading_topic_callback(self,heading_msg):
      if not rospy.is_shutdown():
      self.rbx_navpose_heading_pub.publish(heading_msg)


  #######################
  # Mavlink Interface Methods

  ### Callback to get current state message
  def get_state_callback(self,mavlink_state_msg):
    self.mavlink_state = mavlink_state_msg
    # Update rbx state value
    arm_val = mavlink_state_msg.armed
    if arm_val == True:
      self.state_ind=1
    else:
      self.state_ind=0
    # Update rbx mode value
    mode_val = mavlink_state_msg.mode
    mode_ind=-1
    for ind, mode in enumerate(self.RBX_MODES):
      if mode == mode_val:
        mode_ind=ind
    self.mode_ind=mode_ind   


  ### Function to set mavlink armed state
  def set_mavlink_arm_state(self,arm_value):
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = arm_value
    rospy.loginfo("Updating armed")
    rospy.loginfo(arm_value)
    time.sleep(1) # Give time for other process to see busy
    while self.mavlink_state.armed != arm_value and not rospy.is_shutdown():
      time.sleep(.25)
      self.arming_client.call(arm_cmd)
      rospy.loginfo("Waiting for armed value to set")
      rospy.loginfo("Set Value: " + str(arm_value))
      rospy.loginfo("Cur Value: " + str(self.mavlink_state.armed))

  ### Function to set mavlink mode
  def set_mavlink_mode(self,mode_new):
    new_mode = SetModeRequest()
    new_mode.custom_mode = mode_new
    rospy.loginfo("Updating mode")
    rospy.loginfo(mode_new)
    time.sleep(1) # Give time for other process to see busy
    while self.mavlink_state.mode != mode_new and not rospy.is_shutdown():
      time.sleep(.25)
      self.mode_client.call(new_mode)
      rospy.loginfo("Waiting for mode to set")
      rospy.loginfo("Set Value: " + mode_new)
      rospy.loginfo("Cur Value: " + str(self.mavlink_state.mode))



  ### Callback to get current mavlink battery message
  def get_mavlink_battery_callback(self,battery_msg):
    self.battery_percent = battery_msg.percentage
 

  #######################
  # Mavlink Ardupilot Interface Methods

  ### Function for switching to arm state
  global arm
  def arm(self):
    self.set_mavlink_arm_state(True)

  ### Function for switching to disarm state
  global disarm
  def disarm(self):
    self.set_mavlink_arm_state(False)

  ## Function for sending takeoff command
  global takeoff
  def takeoff(self,timeout_sec):
    cmd_success = True
    self.update_current_errors( [0,0,0,0,0,0,0] )
    start_alt_m = self.current_location_wgs84_geo[2]
    start_alt_goal = start_alt_m + self.takeoff_m
    rospy.loginfo("Sending Takeoff Command to altitude to " + str(self.takeoff_m) + " meters")
    time.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it
    self.takeoff_client(min_pitch=TAKEOFF_MIN_PITCH_DEG,altitude=self.takeoff_m)
    rospy.loginfo("Waiting for takeoff process to complete")
    print_timer=0
    stabilize_timer=0
    alt_error_m = (self.current_location_wgs84_geo[2] - start_alt_goal)
    alt_errors = []
    max_alt_errors = self.rbx_status.error_bounds.max_distance_error_m + abs(alt_error_m)
    timeout_timer = 0
    while max_alt_errors > self.rbx_status.error_bounds.max_distance_error_m and not rospy.is_shutdown():
      rospy.loginfo(max_alt_errors)
      time2sleep = 0.2 # takeoff position check rate
      stabilize_timer = stabilize_timer + time2sleep
      if timeout_timer > timeout_sec:
        rospy.loginfo("Takeoff action timed out")
        cmd_success = False
        break
      else:
        timeout_timer = timeout_timer+time2sleep
      time.sleep(time2sleep) 
      print_timer = print_timer+time2sleep
      alt_error_m = self.current_location_wgs84_geo[2] - start_alt_goal
      self.update_current_errors( [0,0,alt_error_m,0,0,0,0] )
      if print_timer > 1:
        print_timer = 0
        rospy.loginfo("Takeoff errors")
        rospy.loginfo(self.rbx_status.errors_current)
        
      if stabilize_timer > self.rbx_status.error_bounds.max_stabilize_time_s:
        max_alt_errors = max(alt_errors)
        rospy.loginfo("Max alt error over stabilized time")
        rospy.loginfo(max_alt_errors)
        alt_errors = abs(alt_error_m) # reinitialize alt_errors list
        stabilize_timer = 0
      else:
        alt_errors.append(abs(alt_error_m)) # append last
    if cmd_success:
      rospy.loginfo("Takeoff action complete")
    self.update_current_errors( [0,0,0,0,0,0,0] )
    self.update_prev_errors( [0,0,alt_error_m,0,0,0,0] )
    return cmd_success


  ### Function for switching to STABILIZE mode
  global stabilize
  def stabilize(self):
    self.set_mavlink_mode('STABILIZE')
    cmd_success = True
    return cmd_success
      
  ### Function for switching to LAND mode
  global land
  def land(self):
    self.set_mavlink_mode('LAND')
    rospy.loginfo("Waiting for land process to complete and disarm")
    while self.rbx_status.state == 1:
      nepi_ros.sleep(1,10)
    rospy.loginfo("Land process complete")
    success = True
    return success


  ### Function for sending go home command
  global rtl
  def rtl(self):
    self.set_mavlink_mode('RTL')
    success = True
    return success


  ### Function for switching to LOITER mode
  global loiter
  def loiter(self):
    self.set_mavlink_mode('LOITER')
    success = True
    return success


  ### Function for switching to Guided mode
  global guided
  def guided(self):
    self.set_mavlink_mode('GUIDED')
    success = True
    return success

  ### Function for switching back to current mission
  global resume
  def resume(self):
    # Reset mode to last
    rospy.loginfo("Switching mavlink mode from " + self.RBX_MODES[self.rbx_status.mode] + " back to " + self.RBX_MODES[self.rbx_mode_last])
    self.rbx_set_mode(self.rbx_mode_last)
    success = True
    return success


  ### Function for sending set home current
  global sethome_current
  def sethome_current(self):
    rospy.loginfo('Sending mavlink set home current command')
    time.sleep(.1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it
    self.set_home_client(current_gps=True)
    success = True
    return success



  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    rospy.loginfo("Shutting down: Executing script cleanup actions")
    self.stabilize() # Change mode


#########################################
# Main
#########################################
if __name__ == '__main__':
  rospy.loginfo("Starting Ardupilot RBX Driver Script")
  rospy.init_node
  rospy.init_node(name= NEPI_RBX_NODENAME)
  #Launch the node
  current_filename = sys.argv[0].split('/')[-1]
  node_name = current_filename.split('.')[0]
  rospy.loginfo("Launching node named: " + node_name)
  node_class = eval(node_name)
  node = node_class()  
  #Set up node shutdown
  rospy.on_shutdown(node.cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()







