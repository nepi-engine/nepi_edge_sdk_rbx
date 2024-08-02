#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# NEPI RBS Auto Discovery Script for Mavlink devices

import sys
import time
import rospy
import serial
import serial.tools.list_ports
import subprocess
import copy

from nepi_edge_sdk_base import nepi_ros

from mavros_msgs.srv import VehicleInfoGet
from mavros_msgs.msg import VehicleInfo

sys.path.append("/opt/nepi/ros/lib/nepi_drivers_rbx")

#Define Discovery Search Parameters
BAUDRATE_LIST = [57600] # Just one supported baud rate at present

# Globals
mav_port_dict = dict()
mav_ip_dict = dict()

mav_port_entry = {
            "sysid": None,
            "compid": None,     
            "node_name": None,  
            "mavlink_subproc": None,
            "mavqgc_subproc": None,
            "ardu_subproc": None,
            "fgps_subproc": None
}


mav_ip_entry = {
            "sysid": None,
            "compid": None,     
            "node_name": None,  
            "mavlink_subproc": None,
            "mavqgc_subproc": None,
            "ardu_subproc": None,
            "fgps_subproc": None
}


#########################################
# Mavlink Discover Method
#########################################

### Function to try and connect to device and also monitor and clean up previously connected devices
def mavlink_discover(active_port_list):
  global mav_port_dict
  global mav_ip_dict
  
  new_ip_entry = copy.deepcopy(mav_ip_entry)


  node_name = rospy.get_name().split('/')[-1] + '/mavlink_auto_discovery'
  base_namespace = rospy.get_namespace()
  # Find serial ports
  rospy.logdebug(node_name + ": Looking for serial ports on device")
  port_list = []
  ports = serial.tools.list_ports.comports()
  for loc, desc, hwid in sorted(ports):
    rospy.logdebug(node_name + ": Found serial_port at: " + loc)
    port_list.append(loc)
  # Checking for devices on available serial ports
  for port_str in port_list:
    if port_str not in active_port_list:
      found_mavlink = False
      sys_id_mavlink = 0
      found_mavqgc = False
      sys_id_mavqgc = 0
      for baud_int in BAUDRATE_LIST:
        rospy.logdebug(node_name + ": Connecting to serial port " + port_str + " with baudrate: " + str(baud_int))
        try:
          # Try and open serial port
          rospy.logdebug(node_name + ": Opening serial port " + port_str + " with baudrate: " + str(baud_int))
          serial_port = serial.Serial(port_str,baud_int,timeout = 1)
        except Exception as e:
          rospy.logwarn("Mavlink_AD: " + node_name + ": Unable to open serial port " + port_str + " with baudrate: " + str(baud_int) + "(" + str(e) + ")")
          continue
        
        for i in range(0,500): # Read up to 64 packets waiting for heartbeat
          try:
            #serial_port.read_until(b'\xFE', 255) # This is the MAVLINK_1 magic number
            bytes_read = serial_port.read_until(b'\xFD', 280) # MAVLINK_2 packet start magic number, up to MAVLINK_2 max bytes in packet
            bytes_read_count = len(bytes_read)
          except Exception as e:
            rospy.logwarn("Mavlink_AD: read_until() failed (%s)", str(e))
            continue

          if bytes_read_count == 0 or bytes_read_count == 255: # Timed out or read the max mavlink bytes in a packet
            rospy.logdebug('read %d bytes without finding the mavlink packet delimiter... assuming there is no mavlink on this port/baud combo', bytes_read_count)
            break

          try:
            pkt_hdr = serial_port.read(9) # MAVLINK_2 packet header length
          except Exception as e:
            print("read failed (" + str(e) + ")")
            continue

          # Initialize as a non-heartbeat packet
          pkt_len = 255
          comp_id = 255
          msg_id_l = 255

          sys_id = 0
          if len(pkt_hdr) == 9:
            #print(''.join('{:02x}'.format(x) for x in pkt_hdr))
            # This decoding assumes mavlink_2 format packet
            pkt_len = pkt_hdr[0]
            sys_id = pkt_hdr[4]
            comp_id = pkt_hdr[5]
            msg_id_l, msg_id_m, msg_id_h = pkt_hdr[6], pkt_hdr[7], pkt_hdr[8]
          
          # Identify a heartbeat packet by tell-tale signs
          if pkt_len == 9 and msg_id_l == 0x0 and msg_id_m == 0x0 and msg_id_h == 0x0: # Heartbeat message id = 0x00 00 00
            print(str(i) + ": HTBT, sys_id = " + str(sys_id) + ', comp_id = ' + str(comp_id))
            if sys_id > 0 and sys_id < 240:
              found_mavlink = True
              sys_id_mavlink = sys_id
              addr_str = str(sys_id_mavlink)
              rospy.loginfo("Mavlink_AD: Found mavlink device at: " + port_str + "_" + addr_str)
              break
            
          
        # Clean up the serial port
        rospy.logdebug(node_name + ": Closing serial port " + port_str)
        serial_port.close()
        nepi_ros.sleep(1,10)

        # If this is a mavlink, load params and launch the mavros node
        if found_mavlink:
          addr_str = str(sys_id_mavlink)
          port_str_short = port_str.split('/')[-1]
          mavlink_node_name = "mavlink_" + port_str_short
          mavlink_node_namespace = base_namespace + mavlink_node_name
          rospy.loginfo("Mavlink_AD: Starting mavlink node setup: " + mavlink_node_name)
          # Load the proper configs for APM
          rosparam_load_cmd = ['rosparam', 'load', '/opt/ros/noetic/share/mavros/launch/apm_pluginlists.yaml', mavlink_node_namespace]
          subprocess.run(rosparam_load_cmd)
          rosparam_load_cmd = ['rosparam', 'load', '/opt/ros/noetic/share/mavros/launch/apm_config.yaml', mavlink_node_namespace]
          subprocess.run(rosparam_load_cmd)
          # Adjust the timesync_rate to cut down on log noise
          rospy.set_param(mavlink_node_namespace + '/conn/timesync_rate', 1.0)
          # Allow the HIL plugin. Disabled in apm configs for some reason
          plugin_blacklist = rospy.get_param(mavlink_node_namespace + '/plugin_blacklist')
          if 'hil' in plugin_blacklist:
            plugin_blacklist.remove('hil')
            rospy.set_param(mavlink_node_namespace + '/plugin_blacklist', plugin_blacklist)
          
          # Launch Mavlink Node
          rospy.loginfo("Mavlink_AD: Launching mavlink node: " + mavlink_node_name)
          fcu_url = port_str + ':' + str(baud_int)
          gcs_url = "udp://@localhost"
          node_run_cmd = ['rosrun', 'mavros', 'mavros_node', '__name:=' + mavlink_node_name, '_fcu_url:=' + fcu_url, '_gcs_url:=' + gcs_url] 
          mav_subproc = subprocess.Popen(node_run_cmd)

         
          # Start the ardupilot RBX node for this mavlink connection
          ardu_node_name = "ardupilot_" + port_str_short
          rospy.loginfo("Mavlink_AD: " + "Starting ardupilot rbx node: " + ardu_node_name)
          processor_run_cmd = ["rosrun", "nepi_drivers_rbx", "ardupilot_rbx_node.py",
                                "__name:=" + ardu_node_name, f"__ns:={base_namespace}"]
          ardu_subproc = subprocess.Popen(processor_run_cmd)


          #Start the an RBX fake gps for this ardupilot node
          fgps_node_name = "fake_gps_" + port_str_short
          rospy.loginfo("Mavlink_AD: " + "Starting fake gps rbx node: " + fgps_node_name)
          processor_run_cmd = ["rosrun", "nepi_drivers_rbx", "rbx_fake_gps.py",
                                "__name:=" + fgps_node_name, f"__ns:={base_namespace}"]
          fgps_subproc = subprocess.Popen(processor_run_cmd)


          # And make sure it actually starts up fully by waiting for a guaranteed service
          vehicle_info_service_name = mavlink_node_namespace + '/vehicle_info_get'
          try:
            rospy.wait_for_service(vehicle_info_service_name, timeout=10) # TODO: 10 seconds always sufficient for the driver?
          
            # No exception, all good
            active_port_list.append(port_str)

            mav_entry = copy.deepcopy(mav_port_entry)
            mav_entry["sysid"] = sys_id_mavlink
            mav_entry["compid"] =  comp_id   
            mav_entry["node_name"] =  mavlink_node_name
            mav_entry["mavlink_subproc"] = mav_subproc
            mav_entry["ardu_subproc"] = ardu_subproc
            mav_entry["fgps_subproc"] = fgps_subproc

            mav_port_dict[port_str] = mav_entry

            break # Don't check any more baud rates since this one was already successful
          except:
            rospy.logerr("%s: Failed to start %s", node_name, mavlink_node_name)
  
  # Finally check for and purge any nodes no longer running
  port_purge_list = []
  for port in mav_port_dict.keys():
    purge_node = False
    mav_entry = mav_port_dict[port]

    sysid = mav_entry["sysid"] 
    compid = mav_entry["compid"]   
    mavlink_node = mav_entry["node_name"]
    mavlink_subproc = mav_entry["mavlink_subproc"] 
    mavqgc_subproc = mav_entry["mavqgc_subproc"] 
    ardu_subproc = mav_entry["ardu_subproc"] 
    fgps_subproc = mav_entry["fgps_subproc"] 

    full_node_name = base_namespace + '/' + mavlink_node
    
    # Check that the mavlink_node process is still running
    if mavlink_subproc.poll() is not None:
      rospy.logwarn("Mavlink_AD: Node process for %s is no longer running... purging from managed list", mavlink_node)
    #rospy.logwarn("Mavlink_AD: Node process for %s is no longer running... purging from managed list", ardu_node)
      purge_node = True
    # Check that the node's port still exists
    elif port not in active_port_list:
      rospy.logwarn("Mavlink_AD: Port %s associated with node %s no longer detected", port, mavlink_node)
      purge_node = True
    else:
      # Now check that the node is actually responsive
      # Use a service call so that we can provide are assured of synchronous response
      vehicle_info_service_name = full_node_name + '/vehicle_info_get'
      vehicle_info_query = rospy.ServiceProxy(vehicle_info_service_name, VehicleInfoGet)
      try:
        # We don't actually care about the contents of the response at this point, but we might in the future for
        # additional aliveness check logic:
        #response = capability_service()
        vehicle_info_query(sysid=sysid, compid=compid, get_all=False)
 
      except Exception as e: # Any exception indicates that the service call failed
        rospy.logwarn("Mavlink_AD Node %s is no longer responding to vehicle info queries (%s)", mavlink_node, str(e))
        purge_node = True

    if purge_node:
      rospy.logwarn("Mavlink_AD: Purging node %s", mavlink_node)

      if port in active_port_list:
        rospy.logwarn("Mavlink_AD: Removing port %s from active list as part of node purging", port)
        active_port_list.remove(port)

      if mavlink_subproc.poll() is None:
        rospy.logwarn("Mavlink_AD: Issuing sigterm to process for %s as part of node purging", mavlink_node)
        mavlink_subproc.kill()
        # Turns out that is not always enough to get the node out of the ros system, so we use rosnode cleanup, too
        # rosnode cleanup won't find the disconnected node until the process is fully terminated
        try:
          mavlink_subproc.wait(timeout=10)
        except:
          pass        
          

        if mavqgc is not None:
          if mavqgc_subproc.poll() is None:
            rospy.logwarn("Mavlink_AD: Issuing sigterm to process for %s as part of node purging", "mavqgc_node")
            mavqgc_subproc.kill()
            # Turns out that is not always enough to get the node out of the ros system, so we use rosnode cleanup, too
            # rosnode cleanup won't find the disconnected node until the process is fully terminated
            try:
              mavqgc_subproc.wait(timeout=10)
            except:
              pass


        if ardu_subproc.poll() is None:
          rospy.logwarn("Mavlink_AD: Issuing sigterm to process for %s as part of node purging", "ardupilot_node")
          ardu_subproc.kill()
          # Turns out that is not always enough to get the node out of the ros system, so we use rosnode cleanup, too
          # rosnode cleanup won't find the disconnected node until the process is fully terminated
          try:
            ardu_subproc.wait(timeout=10)
          except:
            pass

        if fgps_subproc.poll() is None:
          rospy.logwarn("Mavlink_AD: Issuing sigterm to process for %s as part of node purging", "fgps_gps_node")
          fgps_subproc.kill()
          # Turns out that is not always enough to get the node out of the ros system, so we use rosnode cleanup, too
          # rosnode cleanup won't find the disconnected node until the process is fully terminated
          try:
            fgps_subproc.wait(timeout=10)
          except:
            pass

        
        cleanup_proc = subprocess.Popen(['rosnode', 'cleanup'], stdin=subprocess.PIPE)
        try:
          cleanup_proc.communicate(input=bytes("y\r\n", 'utf-8'), timeout=10)
          cleanup_proc.wait(timeout=10) 
        except Exception as e:
          rospy.logwarn("Mavlink_AD: rosnode cleanup failed (%s)", str(e))

      port_purge_list.append(port) 

    # Clean up the globals  
    for port in port_purge_list:
      del  mav_port_dict[port]
  
  return active_port_list


