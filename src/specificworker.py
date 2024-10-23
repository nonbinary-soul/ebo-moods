#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2024 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import time

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 2000
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True


    @QtCore.Slot()
    def compute(self):
        print('SpecificWorker.compute...')
        self.sadnessMood()
        time.sleep(2)
        return True

    def startup_check(self):
        print(f"Testing RoboCompDifferentialRobot.TMechParams from ifaces.RoboCompDifferentialRobot")
        test = ifaces.RoboCompDifferentialRobot.TMechParams()
        print(f"Testing RoboCompLEDArray.Pixel from ifaces.RoboCompLEDArray")
        test = ifaces.RoboCompLEDArray.Pixel()
        QTimer.singleShot(200, QApplication.instance().quit)

    #########################################

    def turn(self, duration: float, angular_speed: float):
        limit = time.time() + duration
        while time.time() <= limit:
            self.differentialrobot_proxy.setSpeedBase(0, angular_speed)
        self.differentialrobot_proxy.stopBase()

    def turn_full(self):
        self.turn(2, 90)
    def turn_left(self):
        self.turn(0.5, 90)
    def turn_right(self):
        self.turn(0.5, -90)

    def moving_side_to_side(self, times_limit: int): 
        self.turn_left()
        self.turn_right()
        self.turn_right()

        times = 1
        while times <= times_limit: 
            self.turn_left()
            self.turn_left()
            self.turn_right()
            self.turn_right()
            times+=1

        self.turn_left()
    
    def moving_backwards_slowly(self): 
        limit = time.time() + 1.5
        while time.time() <= limit:
            self.differentialrobot_proxy.setSpeedBase(-5,0)  

        self.differentialrobot_proxy.stopBase()

    def turn_back_slowly(self): 
        self.turn(1, 5)
        self.turn(1, 5)

    #########################################

    
    def joyMood(self): 
        self.emotionalmotor_proxy.expressJoy()
        self.differentialrobot_proxy.setSpeedBase(5000, 0)
        time.sleep(0.5)
        self.differentialrobot_proxy.setSpeedBase(-5000, 0)
        time.sleep(0.5)
        self.moving_side_to_side(1)
        
    def sadnessMood(self): 
        self.emotionalmotor_proxy.expressSadness()
        time.sleep(0.5)
        self.moving_backwards_slowly()
        time.sleep(0.5)
        self.turn_back_slowly()

    # def dance(self): 
    #     self.emotionalmotor_proxy.expressJoy()
    #     time.sleep(1)
    #     self.differentialrobot_proxy.

    ######################
    # From the RoboCompDifferentialRobot you can call this methods:
    # self.differentialrobot_proxy.correctOdometer(...)
    # self.differentialrobot_proxy.getBasePose(...)
    # self.differentialrobot_proxy.getBaseState(...)
    # self.differentialrobot_proxy.resetOdometer(...)
    # self.differentialrobot_proxy.setOdometer(...)
    # self.differentialrobot_proxy.setOdometerPose(...)
    # self.differentialrobot_proxy.setSpeedBase(...)
    # self.differentialrobot_proxy.stopBase(...)

    ######################
    # From the RoboCompDifferentialRobot you can use this types:
    # RoboCompDifferentialRobot.TMechParams

    ######################
    # From the RoboCompEmergencyStop you can call this methods:
    # self.emergencystop_proxy.isEmergency(...)

    ######################
    # From the RoboCompEmotionalMotor you can call this methods:
    # self.emotionalmotor_proxy.expressAnger(...)
    # self.emotionalmotor_proxy.expressDisgust(...)
    # self.emotionalmotor_proxy.expressFear(...)
    # self.emotionalmotor_proxy.expressJoy(...)
    # self.emotionalmotor_proxy.expressSadness(...)
    # self.emotionalmotor_proxy.expressSurprise(...)
    # self.emotionalmotor_proxy.isanybodythere(...)
    # self.emotionalmotor_proxy.listening(...)
    # self.emotionalmotor_proxy.pupposition(...)
    # self.emotionalmotor_proxy.talking(...)

    ######################
    # From the RoboCompLEDArray you can call this methods:
    # self.ledarray_proxy.getLEDArray(...)
    # self.ledarray_proxy.setLEDArray(...)

    ######################
    # From the RoboCompLEDArray you can use this types:
    # RoboCompLEDArray.Pixel


