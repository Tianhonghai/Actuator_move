#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import sys
import os
import time
import json
import threading
# import numpy as np
import glob
import collections
import rospy
import math
from kobuki_msgs.msg import SensorState
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import action_client
abs_file = os.path.abspath(os.path.dirname(__file__))
sys.path.append(abs_file + "/../../../lib/comm")
sys.path.append(abs_file + "/../../../lib/log")

from actuator import Actuator
from actuator import ErrorInfo
from actuator import ActuatorCmdType
from proxy_client import PS_Socket
from std_msgs.msg import Float32MultiArray
from rlog import rlog
import numpy as np
log = rlog()

# Define Error code
MOD_ERR_NUM = 3600
MOD_ERR_SELF_OFFSET = 20
E_OK = 0
E_MOD_PARAM = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 1
E_MOD_STATUS = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 2
E_MOD_DRIVER = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 3
E_MOD_EXCEPTION = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 5
E_MOD_ABORT_FAILED = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 6


# Define status
STATUS_UNINIT = 'uninitialized'
STATUS_IDLE = 'idle'
STATUS_BUSY = 'busy'
STATUS_ERROR = 'error'

# command description dict
cmd_description_dict = {
    'cmddescribe': {
        'version': '0.0.1',
        'date': '20200323',
        'time': '11:06:25',
    },
    'cmdlist': [
        {
            'cmd': 'Go',
            'atype': 'motion',
            'params': [
                {
                    'name': 'goal',
                    'type': 'string',
                    'default': 'X',
                    'numberlimit': ['X', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H']
                }
            ],
            'return':{
                'type': 'boolean'
            }
        },        
        {
            'cmd': 'Charge',
            'atype': 'motion',
            'params': [],
            'return':{
                'type': 'boolean'
            }
        },
        {
            'cmd': 'Battery',
            'atype': 'sensing',
            'params': [],
            'return':{
                'type': 'float'
            }
        }
    ]
}

def get_dict_key_value(dict_ins, key, value_type):
    if key in dict_ins:
        value = dict_ins.get(key)
        if isinstance(value, value_type) is False:
            value = None
    else:
        value = None
    return value

class ActuatorMove(Actuator):
    def __init__(self, name, proxy_name, proxy_ip):
        Actuator.__init__(self, name)

        self.data_condition_ = threading.Condition()
        self.enable_timer = True
        self.proxy_ip = proxy_ip
        self.status_ = STATUS_IDLE
        self.statuscode_ = E_OK
        self.data_condition_ = threading.Condition()
        self.railparams=np.zeros(3,dtype=float)
        self.rail_condition_pub = rospy.Publisher('/rail/position_temp', Float32MultiArray, queue_size=20)
        self.battery = 0.


        # 订阅move_base服务器的消息
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        # 设置目标点的位置
        # 在rviz中点击 2D Nav Goal 按键，然后单击地图中一点
        # 在终端中就会看到该点的坐标信息
        self.location = dict()

        self.location['A'] = Pose(Point(0.642, -2.715, 0.000), Quaternion(0.000, 0.000, -0.719, 0.694))
        self.location['B'] = Pose(Point(0.419, -5.492, 0.000), Quaternion(0.000, 0.000, -0.720, 0.692))
        self.location['C'] = Pose(Point(-0.312, -8.745, 0.000), Quaternion(0.000, 0.000, -0.721, 0.692))
        self.location['D'] = Pose(Point(-0.307, -11.476, 0.000), Quaternion(0.000, 0.000, -0.728, 0.684))
        self.location['E'] = Pose(Point(0.943, -8.384, 0.000), Quaternion(0.000, 0.000, 0.674, 0.737))

        self.location['F'] = Pose(Point(2.832, 10.652, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))
        self.location['G'] = Pose(Point(2.840, 11.055, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))
        self.location['H'] = Pose(Point(2.859, 11.380, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))

        self.location['X'] = Pose(Point(0.363, -0.067, 0.000), Quaternion(0.000, 0.000, 0.692, 0.721))

        # 设定下一个目标点  
        self.goal = MoveBaseGoal()  
        self.goal.target_pose.header.frame_id = 'map'
       
        # connect to proxy
        self.pub_socket = PS_Socket(self.proxy_ip)
        self.sub_socket = PS_Socket(self.proxy_ip, self.sub_callback, self)

        self.center_pose_sub = rospy.Subscriber("/mobile_base/sensors/core", SensorState, self.battery_callback)
        self.rate = 20

    def battery_callback(self, core):
        self.battery = core.battery / 10.




    def spinOnce(self):
        r = rospy.Rate(self.rate)
        r.sleep()

    def sim_update(self):
        pass

    def sub_callback(self, caller_args, topic, content):
        print "[", topic, "]: ", content

    # override function
    def sync_cmd_handle(self, msg):
        print "%s syncCmdHandle %s" % (self.name_, msg.cmd)
        is_has_handle = True
        if msg.cmd == "getcmdlist":
            print "{0}:get {1}()".format(self.name_, msg.cmd)
            result_dic = self.get_cmd_list()
            err_info = ErrorInfo(0, "")
            self.reply_result(msg, err_info, result_dic)
        elif msg.cmd == "getstatus":
            result_dic = self.get_status_dict()
            err_info = ErrorInfo(0, "")
            self.reply_result(msg, err_info, result_dic)
        else:
            is_has_handle = False
        return is_has_handle

    # override function
    def async_cmd_handle(self, msg):
        is_has_handle = True
        error_code = 0
        print "%s: asyncCmdHandle " % self.name_
        print "cmd:", msg.cmd
        print "params:", msg.params
        print ""


        if msg.cmd == "Go":
            print "Get cmd Go"
            p0 = get_dict_key_value(msg.params, 'goal', char)
            if p0 is None:
                error_code = E_MOD_PARAM
                error_info = ErrorInfo(error_code, "params [p0] none")
            elif p0 == "A":
                self.goal.target_pose.pose = self.location['A']
                self.goal.target_pose.header.stamp = rospy.Time.now()
                self.move_base.send_goal(self.goal)
            elif p0 == "B":
                self.goal.target_pose.pose = self.location['B']
                self.goal.target_pose.header.stamp = rospy.Time.now()
                self.move_base.send_goal(self.goal)
            elif p0 == "C":
                self.goal.target_pose.pose = self.location['C']
                self.goal.target_pose.header.stamp = rospy.Time.now()
                self.move_base.send_goal(self.goal)
            elif p0 == "D":
                self.goal.target_pose.pose = self.location['D']
                self.goal.target_pose.header.stamp = rospy.Time.now()
                self.move_base.send_goal(self.goal)
            elif p0 == "E":
                self.goal.target_pose.pose = self.location['E']
                self.goal.target_pose.header.stamp = rospy.Time.now()
                self.move_base.send_goal(self.goal)
            elif p0 == "F":
                self.goal.target_pose.pose = self.location['F']
                self.goal.target_pose.header.stamp = rospy.Time.now()
                self.move_base.send_goal(self.goal)
            elif p0 == "G":
                self.goal.target_pose.pose = self.location['G']
                self.goal.target_pose.header.stamp = rospy.Time.now()
                self.move_base.send_goal(self.goal)
            elif p0 == "H":
                self.goal.target_pose.pose = self.location['H']
                self.goal.target_pose.header.stamp = rospy.Time.now()
                self.move_base.send_goal(self.goal)
            else:
                pass
        elif msg.cmd == "Charge":
            self.goal.target_pose.pose = self.location['X']
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.move_base.send_goal(self.goal)
        elif msg.cmd == "Battery":
            percent = ((95*(self.battery - 13.2)) / (16.5 - 14.0)) + 5
            percent = max(min(percent, 100.), 0.)
            return percent

        if True == is_has_handle:
            if 0 == error_code:
                error_info = ErrorInfo(error_code, "")
            elif E_MOD_PARAM == error_code:
                error_info = ErrorInfo(error_code, "params error!")
            else:
                error_info = ErrorInfo(error_code, "execution error!")
            self.reply_result(msg, error_info, None)

        return is_has_handle

    # override function
    def abort_handle(self):
        print "%s: abort_handle ss" % self.name_

    def reset_handle(self):
        print "%s: reset_handle ss" % self.name_

    def get_cmd_list(self):
        return cmd_description_dict

    def get_status_dict(self):
        status_dic = {
            'status': self.get_status(),
            'code':self.get_statuscode(),
        }
        return status_dic

    def set_status(self, status):
        with self.data_condition_:
            self.status_ = status

    def get_status(self):
        with self.data_condition_:
            status = self.status_
        return status

    def set_statuscode(self, statuscode):
        with self.data_condition_:
            self.statuscode_ = statuscode

    def get_statuscode(self):
        with self.data_condition_:
            status = self.statuscode_
        return status
