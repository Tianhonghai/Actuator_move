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
from kobuki_msgs.msg import SensorState, AutoDockingAction, AutoDockingGoal, AutoDockingFeedback, AutoDockingResult
import actionlib
import roslib
import rospy
import actionlib
import tf
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
from random import sample
from math import pow, sqrt
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
# Return goal status as bellow:
# PENDING = 0
# ACTIVE = 1
# PREEMPTED = 2
# SUCCEEDED = 3
# ABORTED = 4
# REJECTED = 5
# PREEMPTING = 6
# RECALLING = 7
# RECALLED = 8
# LOST = 9
cmd_description_dict = {
    'cmddescribe': {
        'version': '0.0.1',
        'date': '20200323',
        'time': '11:06:25',
    },
    'cmdlist': [
        {
            'cmd': 'move',
            'atype': 'motion',
            'params': [
                {
                    'name': 'goal',
                    'type': 'string',
                    'default': 'A'
                }
            ],
        },
        {
            'cmd': 'go',
            'atype': 'motion',
            'params': [
                {
                    'name': 'goal',
                    'type': 'string',
                    'default': 'A',
                    'listlimit': ['A', 'B', 'B1', 'B2', 'B3', 'C', 'D', 'E', 'F', 'G', 'H', 'X', 'Y', 'Z']
                }
            ],
        },
        {
            'cmd': 'charge',
            'atype': 'motion',
            'params': [
                {
                'name': 'percent',
                'type': 'float',
                'default': 80.0,
                'numberlimit': [0.0, 100.0],
                'unit': '%'
                }
            ],
        },
        {
            'cmd': 'battery',
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
    def __init__(self, name, is_simulation, proxy_name, proxy_ip):
        Actuator.__init__(self, name)
        self.is_simulation_ = is_simulation

        self.data_condition_ = threading.Condition()
        self.enable_timer = True
        self.proxy_ip = proxy_ip
        self.status_ = STATUS_IDLE
        self.statuscode_ = E_OK
        self.data_condition_ = threading.Condition()
        self.railparams=np.zeros(3,dtype=float)

        self.battery = 0.0
        self.percent = 0.0

        self.charge_limit = 0.0
        self.charge_reset = False
        self.located = False

        if not self.is_simulation_:
            self.location = dict()

            self.location['X'] = Pose(Point(11.814, -1.608, 0.000), Quaternion(0.000, 0.000, -0.712, 0.701))
            self.location['Y'] = Pose(Point(11.454, -1.575, 0.000), Quaternion(0.000, 0.000, -0.712, 0.701))
            self.location['Z'] = Pose(Point(11.083, -1.586, 0.000), Quaternion(0.000, 0.000, -0.712, 0.701))

            self.location['W'] = Pose(Point(-0.872, -0.055, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))

            self.location['C'] = Pose(
                Point(2.284, -0.332, 0.000), Quaternion(0.000, 0.000, 0.703, 0.710))

            self.location['D'] = Pose(
                Point(-0.893, 0.202, 0.000), Quaternion(0.000, 0.000, 0.703, 0.710))

            self.location['E'] = Pose(
                Point(-4.003, 0.004, 0.000), Quaternion(0.000, 0.000, 0.703, 0.710))

            self.location['F'] = Pose(
                Point(-6.914,  0.191, 0.000), Quaternion(0.000, 0.000, 0.703, 0.710))

            self.location['G'] = Pose(
                Point(-9.889, 0.416, 0.000), Quaternion(0.000, 0.000, 0.703, 0.710))

            self.location['H'] = Pose(
                Point(-12.694,  0.125, 0.000), Quaternion(0.000, 0.000, 0.703, 0.710))

            # self.location['B3'] = Pose(
            #     Point(-9.721, -7.556, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))

            self.location['B3'] = Pose(
                Point(-10.183, -6.556, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))

            # self.location['B2'] = Pose(
            #     Point(-10.183, -5.306, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))

            self.location['B2'] = Pose(
                Point(-10.183, -4.306, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))

            self.location['B1'] = Pose(
                Point(-9.958, -2.205, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))

            self.location['B'] = Pose(
                Point(-6.639, -1.678, 0.000), Quaternion(0.000, 0.000, -0.706, 0.707))

            self.location['A'] = Pose(
                Point(-3.642, -1.826, 0.000), Quaternion(0.000, 0.000, -0.706, 0.707))

        
            # Set move_base client
            self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
            self.goal_code = ''
            self.goal = MoveBaseGoal()
            self.goal.target_pose.header.frame_id = 'map'

            # Set dock_drive_action
            self.auto_docking = actionlib.SimpleActionClient("dock_drive_action", AutoDockingAction)
            self.goal_docking = AutoDockingGoal()
            self.center_pose_sub = rospy.Subscriber("/mobile_base/sensors/core", SensorState, self.battery_callback)

            self.tf_listener = tf.TransformListener()
            while not self.tf_listener.canTransform('map', 'base_footprint', rospy.Time(0.0)):
                log.info("Waiting for location")
                time.sleep(1.0)
            self.located = True
            log.info("Locate successefully, robot will be in idle")



        # connect to proxy
        self.pub_socket = PS_Socket(self.proxy_ip)
        self.sub_socket = PS_Socket(self.proxy_ip, self.sub_callback, self)

        self.rate = 20

    def battery_callback(self, core):
        # self.battery = core.battery / 10.
        percent = ((95*(core.battery / 10. - 13.2)) / (16.5 - 13.2)) + 5
        self.percent = max(min(percent, 100.), 0.)

    def activeCb(self):
        log.info('Move_base: {} Active' .format(self.goal_code))

    def doneCb(self, status, result):
        if status == 3:
            log.info('Move_base: {} Arrived'.format(self.goal_code))
        else:
            log.info('Move_base: {} Aborted'.format(self.goal_code))

    def feedbackCb(self, feedback):
        x1 = self.goal.target_pose.pose.position.x
        x2 = feedback.base_position.pose.position.x
        y1 = self.goal.target_pose.pose.position.y
        y2 = feedback.base_position.pose.position.y
        # print "Distance from %s is %f" % (self.goal_code, ((x1 - x2)**2 + (y1 - y2)**2)**0.5)
        log.info("Distance from {} is {}" .format(self.goal_code, ((x1 - x2)**2 + (y1 - y2)**2)**0.5))
        # Print state of dock_drive module (or node.)

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
        value_ret = None
        self.goal_code = ''
        print "%s: asyncCmdHandle " % self.name_
        print "cmd:", msg.cmd
        print "params:", msg.params
        print ""
        error_info = ErrorInfo(0, "")


        if msg.cmd == "go" or msg.cmd == "move":
            log.info("Get cmd go")

            if self.is_simulation_:
                error_code = E_OK
                error_info = ErrorInfo(error_code, "")
            
            # No need to judge battery in new strategy, just keep it for interface. It is judged in project
            elif self.percent < 0.0:
                error_code = E_MOD_STATUS
                log.info("Battery is low, omit cmd go")
            else:
                while not self.move_base.wait_for_server(rospy.Duration(1.0)):
                    if rospy.is_shutdown():
                        return
                    log.info("Waiting for move_base server...")
                log.info("Move_base server connected")
                self.goal_code = get_dict_key_value(msg.params, 'goal', (str, unicode))
                p0 = self.goal_code
                log.info("Param parsed is {}".format(p0))
                self.charge_reset = True

                # make sure action server get right state after last move. TODO: lookup if has api in action server
                rospy.sleep(1.0)

                self.goal.target_pose.header.stamp = rospy.Time.now()
                if p0 is None:
                    error_code = E_MOD_PARAM
                    error_info = ErrorInfo(error_code, "params [p0] none")
                elif p0 in ['A', 'B', 'B1', 'B2', 'B3', 'C', 'D', 'E', 'F', 'G', 'H', 'X', 'Y', 'Z']:
                    log.info("Get {}".format(p0))
                    self.goal.target_pose.pose = self.location[p0]
                    self.move_base.send_goal(
                        self.goal, self.doneCb, self.activeCb, self.feedbackCb)
                    self.move_base.wait_for_result()
                    status = self.move_base.get_state()
                    log.info('Final status is {}'.format(status))
                    if status != 3:
                        error_code = E_MOD_EXCEPTION
                        error_info = ErrorInfo(error_code, "params {} done error".format(p0))
                        log.error("params {} done error".format(p0))
                        self.move_base.cancel_goal()
                    
                else:
                    error_code = E_MOD_PARAM
                    error_info = ErrorInfo(error_code, "params error")
        elif msg.cmd == "charge":
            log.info("Get cmd charge")
            self.goal_code = 'W'
            self.charge_reset = False
            self.charge_limit = get_dict_key_value(msg.params, 'percent', (int,float))
            log.info("charge limit is {}".format(self.charge_limit))

            if self.is_simulation_:
                error_code = E_OK
                error_info = ErrorInfo(error_code, "")
            elif self.charge_limit < self.percent:
                error_code = E_OK
                error_info = ErrorInfo(error_code, "")
                log.info("Battery percent is higher than limit given, omit charge cmd")
            else:
                # Navigation to point by laser in front of dock station
                time.sleep(1.0)
                while not self.move_base.wait_for_server(rospy.Duration(1.0)):
                    if rospy.is_shutdown():
                        return
                    log.info("Waiting for move_base server...")
                log.info("Move_base server connected")
                self.goal.target_pose.header.stamp = rospy.Time.now()
                self.goal.target_pose.pose = self.location['W']
                self.move_base.send_goal(self.goal, self.doneCb, self.activeCb, self.feedbackCb)
                self.move_base.wait_for_result()
                status = self.move_base.get_state()
                log.info('Final status is {}'.format(status))
                if status != 3:
                    error_code = E_MOD_EXCEPTION
                    error_info = ErrorInfo(error_code, "charge laser navigation done error")
                    log.error("charge laser navigation done error")
                else:
                    # Navigation by ir to dock station
                    while not self.auto_docking.wait_for_server(rospy.Duration(1.0)):
                        if rospy.is_shutdown():
                            return
                        log.info("Waiting for dock_drive_action server...")
                    log.info("Dock_drive_action server connected")
                    self.auto_docking.send_goal(self.goal_docking)
                    self.auto_docking.wait_for_result()
                    status = self.auto_docking.get_state()
                    if status != 3:
                        error_code = E_MOD_EXCEPTION
                        error_info = ErrorInfo(error_code, "charge ir navigation done error")
                        log.error("Charge Ir navigation done error in charge cmd")
                        self.auto_docking.cancel_goal()
                    else:
                        # To diff stopping charge from 'reset/abort handle, and charged to limit' or 'get new goal in cmd go'
                        self.goal_code = ''

                        # Block 1. when percent is lower than limit. 2. No reset cmd
                        while self.percent < self.charge_limit and not self.charge_reset:
                            log.info("Charging, percent is {}".format(self.percent))
                            time.sleep(1.0)

                        if not self.charge_reset:
                            log.info("Charge done, moving to standby")

                            # After charge done, move robot a little behind and turn 180 degree to standby
                            self.goal.target_pose.header.stamp = rospy.Time.now()
                            while not self.move_base.wait_for_server(rospy.Duration(1.0)):
                                if rospy.is_shutdown():
                                    return
                                log.info("Waiting for move_base server...")
                            log.info("Move_base server connected")
                            self.goal.target_pose.pose = Pose(
                                Point(-0.591, -0.074, 0.000),
                                Quaternion(0.000, 0.000, 1.000, 0.000))
                            self.move_base.send_goal(self.goal, self.doneCb, self.activeCb, self.feedbackCb)
                            self.move_base.wait_for_result()
                            status = self.move_base.get_state()
                            if status != 3:
                                error_code = E_MOD_EXCEPTION
                                error_info = ErrorInfo(error_code, "charge laser navigation done error")
                                log.error("Laser navigation done error in charge cmd")
                                self.move_base.cancel_goal()
                        self.charge_reset = False

        elif msg.cmd == "battery":
            if self.is_simulation_:
                value_ret = 60.0
            else:
                value_ret = self.percent
                log.info("Battery is {} percents".format(value_ret))
        else:
            is_has_handle = False

        if True == is_has_handle:
            if 0 == error_code:
                error_info = ErrorInfo(error_code, "")
            elif E_MOD_PARAM == error_code:
                error_info = ErrorInfo(error_code, "params error!")
            elif E_MOD_STATUS == error_code:
                error_info = ErrorInfo(error_code, "battery is low")
            elif E_MOD_EXCEPTION == error_code:
                pass
            else:
                error_info = ErrorInfo(error_code, "execution error!")
            self.reply_result(msg, error_info, value_ret)

        return is_has_handle

    # override function
    def abort_handle(self):
        log.info("{}: abort_handle ss".format(self.name_))
        self.charge_reset = True
        if not self.is_simulation_:
            self.move_base.cancel_goal()

    def reset_handle(self):
        log.info("{}: reset_handle ss".format(self.name_))
        self.charge_reset = True
        if not self.is_simulation_:
            self.move_base.cancel_goal()

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
            if not self.is_simulation_:
                while not self.located:
                    time.sleep(1.0)
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
