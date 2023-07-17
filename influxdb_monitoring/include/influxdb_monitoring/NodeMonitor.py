#!/usr/bin/env python
# -*- coding: utf-8 -*-

# need to install influxdb_client via pip

import rosnode
import rospy
import time
import argparse
import json
import subprocess
import rospkg

import influxdb_client
from influxdb_monitoring.InfluxDBConfig import setConfig
import influxdb_monitoring.InfluxDBConfig as InfluxDBConfig

check_every_x_seconds = 1
display_every_x_seconds = 30
restart_after_x_cycles = 10
max_node_state_percistency = 99
restart_failed_nodes = True
session_name = ""
tmux = "byobu"
#source = "/home/kmr/.bashrc"

node_dict = {}
node_list = {}
window_dict = {}
node_states = []
node_state_percistency = []

robot_name = None
def setRobotName(robot_name_):
    """
    Specify a robot name. All InfluxDB updates have this name as a tag.
    """
    global robot_name
    robot_name = robot_name_

def writeToInfluxDB(state, node_name):
    """
    Writes the state of one node to the database.
    
    Args:
    - state (bool): Current state of the node
    - node_name (str): name of the node
    
    Returns: None
    """
    #print("  r: %.2f, min/max:[%.4f; %.4f], stddev: %.4f   n: %d" % (topic_feedback[0], topic_feedback[1], topic_feedback[2], topic_feedback[3], topic_feedback[4]))
    try:
        point = (
            influxdb_client.Point("ros_nodes")
            .tag("node", node_name)
            .field("up", state)
        )
        if not robot_name is None:
            point.tag("robot", robot_name)
        InfluxDBConfig.write_api.write(bucket=InfluxDBConfig.bucket_g, org=InfluxDBConfig.org_g, record=point)
    except KeyboardInterrupt:
        raise KeyboardInterrupt
    except Exception as e:
        rospy.loginfo("Service call failed: %s"%e)


def writeNodeStatesInflux():
    """
    Iterates over the nodes, saves their state to the database.
    """
    node_self = rospy.get_name()
    for i in range(len(node_list)):
        node = node_list[i]
        state = node_states[i]
        if (node != node_self):
            writeToInfluxDB(state==1, node)
    return




def stateName(state):
    """
    Matches a node state to a string message.
    Args:
    - state (int): state of the node
    
    Returns: String describing the state.
    """
    if state == 1:
        return 'running'
    elif state == 0:
        return 'stopped'
    elif state == -1:
        return 'restarting'
    else:
        return 'undefined'


def load_stuff(skip_parse = False, start_file_ = None, file_pkg_  = None):
    """
    Loads a configuration from a JSON file
    
    Args:
    - skip_parse (bool): Skips parsing of the command line arguments if True. Uses following parameters instead (default False)
    - start_file_ (str): File defining the configuration 
    - file_pkg_ (str): package where the file is located (resolved with rospack)
    
    Returns: args
    """
    global node_dict
    global node_list
    global node_states
    global node_state_percistency
    global window_dict
    global tmux
    global session_name
    if not skip_parse:
        parser = argparse.ArgumentParser(
                        prog = 'ProgramName',
                        description = 'What the program does',
                        epilog = 'Text at the bottom of help')
        parser.add_argument("--startFile", type=str, default="config/node_launch_byobu_mapping.json")
        parser.add_argument("--filePkg", type=str, default="lta_monitoring")
        parser.add_argument("--nodeWatcher", type=str, default="/node2influxdb")
        args, unknown = parser.parse_known_args()
        start_file = args.startFile
        file_pkg = args.filePkg
    elif not start_file_ is None and not file_pkg_ is None:
        start_file = start_file_
        file_pkg = file_pkg_
        args = None
    else:
        print("problem during load_stuff.")
        rospy.logerr("problem during load_stuff.")
        raise Exception
    try:
        rospack = rospkg.RosPack()
        package_dir = rospack.get_path(file_pkg)
        full_filename = package_dir + "/" + start_file
        print(full_filename)
    except:
        print("error while parsing pose of interest file (--startFile and --filePkg)")
        return

    with open(full_filename) as json_file:
        data = json.load(json_file)
    try:
        node_dict = data["nodes"]
        if restart_failed_nodes:
            session_name = data["session_name"]
            tmux = data["tmux"]
            window_dict = data["windows"]
    except:
        print("Could not read session name/tmux/source/node_dict from file %s" % full_filename)
        return False
    node_list = list(node_dict.keys())
    node_states = [0*x for x in range(len(node_list))]
    node_state_percistency = [1+0*x for x in range(len(node_list))]
    return args

def restartNode(node_name):
    """
    Sequence restarting a node in the respective byobu window.
    
    Args:
    - node_name (str): name of the node
    
    Returns: None
    """
    node_window = ""
    try:
        node_window = node_dict[node_name]["window"]
    except:
        pass
    if (node_window == ""):
        rospy.logerr("could not resolve node name to window")
        return
    restart_command = ""
    try:
        restart_command = window_dict[node_window]["restart_cmd"]
    except:
        pass
    if (restart_command == ""):
        try:
            restart_command = window_dict[node_window]["start_cmd"]
        except:
            pass
    if (restart_command == ""):
        rospy.logerr("Could not resolve node to restart command")
        return
    rospy.loginfo("%s: cancelling existing command" % node_name)
    try:
        start_args = [tmux,  "send-keys", "-t", "%s:%s" % (session_name, node_window), "C-c"]
        subprocess.run(start_args)
        start_args = [tmux,  "send-keys", "-t", "%s:%s" % (session_name, node_window), "C-c"]
        subprocess.run(start_args)
        time.sleep(1)
        start_args = [tmux,  "send-keys", "-t", "%s:%s" % (session_name, node_window), "C-c"]
        subprocess.run(start_args)
    except KeyboardInterrupt:
        raise KeyboardInterrupt
    except:
        rospy.logerr("failed during subprocess calling")
        return
    time.sleep(2)

    rospy.loginfo("restarting %s" % node_name)
    try:
        start_args = [tmux,  "send-keys", "-t", "%s:%s" % (session_name, node_window), restart_command, "C-m"]
        subprocess.run(start_args)
    except KeyboardInterrupt:
        raise KeyboardInterrupt
    except:
        rospy.logerr("failed during subprocess calling")
        return
    return


def checkNodes():
    """
    Checks if a node is alive:
    First asks ros for current registered nodes, then pings them to see if they are responsive.

    Node is marked as stopped if either not registered or not pingable.
    """
    global node_states
    registered_nodes = rosnode.get_node_names()
    for i in range(len(node_list)):
        node = node_list[i]
        state = False
        if (node in registered_nodes):
            state = rosnode.rosnode_ping(node, verbose=False, max_count=1)
        if state:
            new_state = 1
        else:
            new_state = 0
        if new_state == 0 and node_states[i] == 1:
            rospy.logerr("Node %s stopped" % node)
            writeToInfluxDB(False, node)
        if (new_state == node_states[i]):
            node_state_percistency[i] += 1
        else:
            node_state_percistency[i] = 0
        if (node_state_percistency[i] > max_node_state_percistency):
            node_state_percistency[i] = max_node_state_percistency
        if (not state and node_state_percistency[i] > restart_after_x_cycles and restart_failed_nodes):
            restartNode(node)
            new_state = -1
            node_state_percistency[i] = 0
        node_states[i] = new_state
    return

def printNodeStates():
    """
    Prints the current state of the nodes.
    """
    msg_string = "\nstate (cycles)\t| name"
    for i in range(len(node_list)):
        node = node_list[i]
        state = node_states[i]
        percistency = node_state_percistency[i]
        msg_string += "\n%s (%4d ) | %s" % (stateName(state), percistency, node)
    rospy.loginfo(msg_string)
    return

def monitoring():
    rate = rospy.Rate(1.0/check_every_x_seconds) # ROS Rate every 60s
    last_write = rospy.Time(0)
    while not rospy.is_shutdown():
        try:
            checkNodes()
            if (any([p == 0 for p in node_state_percistency]) or (rospy.Time.now() - last_write).to_sec() > display_every_x_seconds):
                last_write = rospy.Time.now()
                printNodeStates()
                writeNodeStatesInflux()
        except rosnode.ROSNodeIOException as e:
            rospy.logwarn(e)
            break
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except Exception as e:
            print(type(e))
            rospy.logwarn(e)
        rate.sleep()

def startMainMonitor(check_every_x_seconds_=2, write_state_every_x_seconds_=60, restart_after_x_cycles_ = 10,
                     max_node_state_percistency_ = 999, restart_failed = True):
    """
    Function starts the Main node monitoring, observing the nodes specified in the loaded config.
    Only returns on error.
    
    Args:
    - check_every_x_seconds_ (float): delay between subsequent node checks (-> one cycle) (default 2 seconds)
    - write_state_every_x_seconds_ (float): how often the state should be written to the data base, immediatly updated on change! (default 60 seconds)
    - restart_after_x_cycles_ (int): restarts a node after this many cycles of inactivity. (default 10 cycles = 20 seconds)
    - max_node_state_percistency_ (int): maximum of shown node persistency, i.e. number of cycles in current state (in print statement)
    - restart_failed (bool): Flag indicating if failed nodes should be restarted. Useful if only monitoring is required (default True)
    
    Returns: None
    """

    global check_every_x_seconds, display_every_x_seconds, restart_after_x_cycles, max_node_state_percistency, restart_failed_nodes
    check_every_x_seconds = check_every_x_seconds_
    display_every_x_seconds = write_state_every_x_seconds_
    restart_after_x_cycles = restart_after_x_cycles_
    max_node_state_percistency = max_node_state_percistency_
    restart_failed_nodes = restart_failed
    
    load_stuff()
    monitoring()
    
def startSecondaryMonitor(main_monitor_node_name="/node2influxdb", check_every_x_seconds_=2, write_state_every_x_seconds_=60,
                          restart_after_x_cycles_ = 10, max_node_state_percistency_ = 999, restart_failed = True):
    """
    Function starts the Main node monitoring, observing the nodes specified in the loaded config.
    Only returns on error.
    
    Args:
    - main_monitor_node_name (str): name of the accompanying MainMonitor node (Default /node2influxdb)
    - check_every_x_seconds_ (float): delay between subsequent node checks (-> one cycle) (default 2 seconds)
    - write_state_every_x_seconds_ (float): how often the state should be written to the data base, immediatly updated on change! (default 60 seconds)
    - restart_after_x_cycles_ (int): restarts a node after this many cycles of inactivity. (default 10 cycles = 20 seconds)
    - max_node_state_percistency_ (int): maximum of shown node persistency, i.e. number of cycles in current state (in print statement)
    - restart_failed (bool): Flag indicating if failed nodes should be restarted (default True)
    
    Returns: None
    """
    global check_every_x_seconds, display_every_x_seconds, restart_after_x_cycles, node_list, max_node_state_percistency, restart_failed_nodes
    check_every_x_seconds = check_every_x_seconds_
    display_every_x_seconds = write_state_every_x_seconds_
    restart_after_x_cycles = restart_after_x_cycles_
    max_node_state_percistency = max_node_state_percistency_
    restart_failed_nodes = restart_failed
    
    load_stuff()
    node_list = [main_monitor_node_name]
    monitoring()
