#! /usr/bin/env python
# -*- coding: utf-8 -*-

from abc import ABC, abstractmethod
import threading
import datetime
import time

import rospy
import actionlib
from std_msgs.msg import String

import sys

class Command(ABC):
    command_prev_id = ""
    command_id = ""
    command_next_id = ""
    command_auth_info = ""
    command_content = ""
    pass

class Client(ABC):
        
    def __init__(self, node_name : str):
        
        rospy.init_node(name=node_name, anonymous=False, log_level=rospy.INFO, disable_signals=False)
        self.robot_type = rospy.get_param('~robot_type','type1')

        self.last_connection_timestamp = time.mktime(datetime.datetime.now().timetuple())
        self.last_connection_timeout = 10

        self.used_ids = set()
        self.sent_command_cache = dict()
        self.controlled_node_id = ""
        self.last_command_id = ""
        self.next_command_id = ""
    
    @abstractmethod
    def connectivityCheck(self, controlled_node_id : str):
        pass
    
    @abstractmethod
    def identification(self, controlled_node_id : str):
        pass
    
    @abstractmethod
    def authentication(self, controlled_node_id : str):
        pass
    
    @abstractmethod
    def take_control(self, controlled_node_id : str):
        pass
    
    @abstractmethod
    def _generateCommand(self) -> Command:
        pass
    
    @abstractmethod
    def sendCommand(self, controlled_node_id : str):
        pass

    @abstractmethod
    def execute(self, controlled_node_id : str):
        pass

    @abstractmethod
    def statusRequest(self, controlled_node_id : str):
        pass

    @abstractmethod
    def sendErrorMsg(self, level : str, content : str):
        pass

    @abstractmethod
    def errorHandler(self, controlled_node_id : str):
        pass

    @abstractmethod
    def releaseControl(self, controlled_node_id : str):
        pass