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

import rospy
import serial
import serial.tools.list_ports
import subprocess

from mavros_msgs.srv import VehicleInfoGet

#Define Discovery Search Parameters
BAUDRATE_LIST = [57600] # Just one supported baud rate at present

# Globals
# TODO: These should all be replaced with a single list of data structs (e.g., dictionaries)
mavlink_node_list = []
mavlink_subproc_list = []
mavlink_port_list = []
mavlink_sysid_list = []
mavlink_compid_list = []

#########################################
# Mavlink Discover Method
#########################################

### Function to try and connect to device and also monitor and clean up previously connected devices
def mavlink_discover(active_port_list):
  global mavlink_node_list
  global mavlink_subproc_list
  global mavlink_port_list
  global mavlink_sysid_list
  global mavlink_compid_list

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
      found_heartbeat = False
      for baud_int in BAUDRATE_LIST:
        rospy.logdebug(node_name + ": Connecting to serial port " + port_str + " with baudrate: " + str(baud_int))
        try:
          # Try and open serial port
          rospy.logdebug(node_name + ": Opening serial port " + port_str + " with baudrate: " + str(baud_int))
          serial_port = serial.Serial(port_str,baud_int,timeout = 1)
        except Exception as e:
          rospy.logwarn(node_name + ": Unable to open serial port " + port_str + " with baudrate: " + str(baud_int) + "(" + str(e) + ")")
          continue
        
        for i in range(0,64): # Read up to 64 packets waiting for heartbeat
          try:
            #serial_port.read_until(b'\xFE', 255) # This is the MAVLINK_1 magic number
            bytes_read = serial_port.read_until(b'\xFD', 280) # MAVLINK_2 packet start magic number, up to MAVLINK_2 max bytes in packet
            bytes_read_count = len(bytes_read)
          except Exception as e:
            rospy.logwarn("%s: read_until() failed (%s)", node_name, str(e))
            continue

          if bytes_read_count == 0 or bytes_read_count == 255: # Timed out or read the max mavlink bytes in a packet
            rospy.logdebug('%s: read %d bytes without finding the mavlink packet delimiter... assuming there is no mavlink on this port/baud combo', node_name, bytes_read_count)
            break

          try:
            pkt_hdr = serial_port.read(9) # MAVLINK_2 packet header length
          except Exception as e:
            print("read failed (" + str(e) + ")")
            continue

          # Initialize as a non-heartbeat packet
          pkt_len = 255
          sys_id = 255
          comp_id = 255
          msg_id_l = 255

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
            found_heartbeat = True
            break
          
        # Clean up the serial port
        rospy.logdebug(node_name + ": Closing serial port " + port_str)
        serial_port.close()
        
        # If this is a mavlink, load params and launch the mavros node
        if found_heartbeat:
          addr_str = str(sys_id)
          rospy.loginfo(node_name + ": Found mavlink device at: " + addr_str)
          port_str_short = port_str.split('/')[-1]
          mavlink_node_name = "mavlink_" + port_str_short + "_" + addr_str
          mavlink_node_namespace = base_namespace + mavlink_node_name
          
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
        
          fcu_url = port_str + ':' + str(baud_int)
          node_run_cmd = ['rosrun', 'mavros', 'mavros_node', '__name:=' + mavlink_node_name, '_fcu_url:=' + fcu_url] 
          p = subprocess.Popen(node_run_cmd)

          # And make sure it actually starts up fully by waiting for a guaranteed service
          vehicle_info_service_name = mavlink_node_namespace + '/vehicle_info_get'
          try:
            rospy.wait_for_service(vehicle_info_service_name, timeout=10) # TODO: 10 seconds always sufficient for the driver?
          
            # No exception, all good
            active_port_list.append(port_str)
            mavlink_port_list.append(port_str)
            mavlink_node_list.append(mavlink_node_name)
            mavlink_subproc_list.append(p)
            mavlink_sysid_list.append(sys_id)
            mavlink_compid_list.append(comp_id)
            break # Don't check any more baud rates since this one was already successful
          except:
            rospy.logerr("%s: Failed to start %s", node_name, mavlink_node_name)
  
  # Finally check for and purge any nodes no longer running
  for i,node in enumerate(mavlink_node_list):
    purge_node = False
    
    subproc = mavlink_subproc_list[i]
    port = mavlink_port_list[i]
    sysid = mavlink_sysid_list[i]
    compid = mavlink_compid_list[i]
    full_node_name = base_namespace + '/' + node
    
    # Check that the node process is still running
    if subproc.poll() is not None:
      rospy.logwarn('%s: Node process for %s is no longer running... purging from managed list', node_name, node)
      purge_node = True
    # Check that the node's port still exists
    elif port not in active_port_list:
      rospy.logwarn('%s: Port %s associated with node %s no longer detected', node_name, port, node)
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
        rospy.logwarn('%s: Node %s is no longer responding to vehicle info queries (%s)', node_name, node, str(e))
        purge_node = True

    if purge_node:
      rospy.logwarn('%s: Purging node %s', node_name, node)

      if port in active_port_list:
        rospy.logwarn('%s: Removing port %s from active list as part of node purging', node_name, port)
        active_port_list.remove(port)

      if subproc.poll() is None:
        rospy.logwarn('%s: Issuing sigterm to process for %s as part of node purging', node_name, node)
        subproc.kill()
        # Turns out that is not always enough to get the node out of the ros system, so we use rosnode cleanup, too
        # rosnode cleanup won't find the disconnected node until the process is fully terminated
        try:
          subproc.wait(timeout=10)
        except:
          pass
        
        cleanup_proc = subprocess.Popen(['rosnode', 'cleanup'], stdin=subprocess.PIPE)
        try:
          cleanup_proc.communicate(input=bytes("y\r\n", 'utf-8'), timeout=10)
          cleanup_proc.wait(timeout=10) 
        except Exception as e:
          rospy.logwarn('%s: rosnode cleanup failed (%s)', node_name, str(e))
            
      # Clean up the globals  
      del mavlink_port_list[i]
      del mavlink_node_list[i]
      del mavlink_subproc_list[i]
      del mavlink_sysid_list[i]
      del mavlink_compid_list[i]
  
  return active_port_list
