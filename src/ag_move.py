#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import sys
import os
import time
import signal
import rospy
import json

abs_file = os.path.abspath(os.path.dirname(__file__))
sys.path.append(abs_file + "/../../../lib/adapter")
sys.path.append(abs_file + "/../../../lib/log")
sys.path.append(abs_file + "/../../../lib/comm")
# print sys.path

from actuator import Adapter
from actuator_move import ActuatorMove
from rlog import rlog


proxy_name = 'ag_move'
actuator_1_name = 'move'
config_json_file_path = "/opt/knowin/configs/config.json"

log = rlog()


def exit_handle(signum, frame):
    print 'You choose to stop me.'
    sys.exit()

def json_loads(json_str):
    try:
        strings = json.loads(json_str, encoding='utf-8')
    except:
        log.error("Parsing json string error! json_str:{}".format(json_str))
        return
    else:
        return strings

def get_config_unit(function_name, module_name, socket_name):
    is_exist = os.path.exists(config_json_file_path)
    if is_exist is False:
        log.error("can not find config file! path:{}".format(config_json_file_path))
        return None

    file_config = open(config_json_file_path, "r")
    json_value = file_config.read()

    config_dict_list = json_loads(json_value)

    if config_dict_list is None:
        return None

    if isinstance(config_dict_list, list) is False:
        return None

    for config_dict in config_dict_list:
        if isinstance(config_dict, dict) is False:
            continue
        if 'name' not in config_dict:
            continue
        if function_name != config_dict.get('name'):
            continue
        if module_name not in config_dict:
            continue
        unit_dict_list = config_dict.get(module_name)

        if isinstance(unit_dict_list, list) is False:
            continue
        for unit_dict in unit_dict_list:
            if isinstance(unit_dict, dict) is True:
                if 'socket' in unit_dict:
                    if socket_name == unit_dict.get('socket'):
                        return unit_dict
    return None

# value_type: int, float, str, bool, list, dict
def get_dict_key_value(dict_ins, key, value_type):
    if key in dict_ins:
        value = dict_ins.get(key)
        if isinstance(value, value_type) is False:
            value = None
    else:
        value = None
    return value

def get_log_level(proxy_name):
    log_level = None
    unit_dict = get_config_unit('application', 'actuator', proxy_name)
    if unit_dict is None:
        log_level = None
    else:
        log_level = get_dict_key_value(unit_dict, 'loglevel', (str, unicode))
    if log_level is None:
        log.warn("read config file fault, use default level [info]")
        log_level = 'info'
    else:
        log_level = log_level.encode('utf-8')
    return log_level


if __name__ == "__main__":
    str_actuator_name = None
    str_proxy_name = None
    str_sim = None
    is_simulation = False
    proxy_ip = "127.0.0.1"


    # argv params parse
    argv_num = len(sys.argv)
    argv_index = 1
    while argv_index < argv_num:
        if '--proxy' == sys.argv[argv_index]:
            if argv_index + 1 < argv_num:
                if sys.argv[argv_index + 1].find('--') != 0:
                    print "--proxy:", sys.argv[argv_index + 1]
                    str_proxy_name = sys.argv[argv_index + 1]
                    argv_index = argv_index + 1
        elif '--sim' == sys.argv[argv_index]:

            if argv_index + 1 < argv_num:
                if sys.argv[argv_index + 1].find('--') != 0:
                    print "--sim:", sys.argv[argv_index + 1]
                    str_sim = sys.argv[argv_index + 1]
                    argv_index = argv_index + 1
        else:
            print "unknow params:", sys.argv[argv_index]
        argv_index = argv_index + 1

    if str_actuator_name is not None:
        print "actuator name change to:", str_actuator_name
        actuator_1_name = str_actuator_name

    # proxy name rename
    if str_proxy_name is not None:
        print "adapter proxy name change to:", str_proxy_name
        proxy_name = str_proxy_name

    log.setModuleName(proxy_name)
    log_level = get_log_level(proxy_name)
    log.warn("set log level to [{}]".format(log_level))
    log.set_priority(log_level)
    # simulation parse
    if str_sim is not None:
        str_sim_list = str_sim.split(':')
        for sim_name in str_sim_list:
            if actuator_1_name == sim_name:
                print actuator_1_name, " will set simulation mode!"
                is_simulation = True
            if 'all' == sim_name:
                print "All will set simulation mode!"
                is_simulation = True
                break
    else:
        rospy.init_node("ag_move_node")

    # register exit function for exit by 'ctrl+c'
    signal.signal(signal.SIGINT, exit_handle)
    signal.signal(signal.SIGTERM, exit_handle)

    # create adapter and special actuator
    adapter = Adapter(proxy_name)

    unit_dict = adapter.get_config_unit('application', 'proxy', 'proxy')
    if unit_dict is None:
        proxy_ip = "localhost"
    else:
        proxy_ip = adapter.get_dict_key_value(unit_dict, 'ip', (str, unicode))
    actuator_move = ActuatorMove(actuator_1_name, proxy_name, proxy_ip)
    # register actuator to adapter
    adapter.reg_actuator(actuator_move)

    adapter.set_status('idle')
    while True:
        time.sleep(1)
