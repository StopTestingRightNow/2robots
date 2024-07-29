#! /usr/bin/env python
# -*- coding: utf-8 -*-

from abc import ABC, abstractmethod
import threading
import queue
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

class Server(ABC):
        
    def __init__(self, node_name : str):
        
        rospy.init_node(name=node_name, anonymous=False, log_level=rospy.INFO, disable_signals=False)
        self.robot_type = rospy.get_param('~robot_type','type1')
    
        self.last_connection_timestamp = time.mktime(datetime.datetime.now().timetuple())
        self.last_connection_timeout = 10

        self.used_ids = set()
        self.command_sequence = []
        self.incoming_command_queue = queue.Queue()
        self.sent_command_cache = dict()
        self.controller_node_id = ""
        self.next_command_id_expected = ""
        self.curr_state = (0,0,0,0)
    
    @abstractmethod
    def connectivityCheck(self, controller_node_id : str):
        pass
    
    @abstractmethod
    def identification(self, controller_node_id : str):
        pass
    
    @abstractmethod
    def authentication(self, controller_node_id : str):
        pass
    
    @abstractmethod
    def authorisation(self, controller_node_id : str, command : Command):
        pass

    @abstractmethod
    def giveControl(self, controller_node_id : str):
        pass

    @abstractmethod
    def receiveCommand(self, controller_node_id : str, command: Command):
        pass

    @abstractmethod
    def emergencyStop(self):
        pass

    @abstractmethod
    def emergencyReset(self):
        pass

    @abstractmethod
    def processCommand(self):
        pass

    @abstractmethod
    def commandSanityCheck(self, command: Command):
        pass

    @abstractmethod
    def _updateCurrState(self):
        pass

    @abstractmethod
    def _statePrediction(self, commandIndex : int):
        pass

    @abstractmethod
    def sequenceSanityCheck(self):
        pass

    @abstractmethod
    def sendErrorMsg(self, level : str, content : str):
        pass

    @abstractmethod
    def execute(self):
        pass

    @abstractmethod
    def statusReport(self, controller_node_id : str):
        pass

    @abstractmethod
    def releaseControl(self):
        pass