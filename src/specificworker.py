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
        self.NUM_LEDS = 52
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
        self.expressJoy()
        time.sleep(2)
        self.expressSadness()
        time.sleep(2)
        self.expressFear()
        time.sleep(2)
        self.expressSurprise()
        time.sleep(2)
        self.expressAnger()
        time.sleep(2)
        self.expressDisgust()
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
    
    def moving_straight(self, duration: float, speed: int): 
        limit = time.time() + duration
        while time.time() <= limit:
            self.differentialrobot_proxy.setSpeedBase(speed, 0)
        self.differentialrobot_proxy.stopBase()

    def jolts(self, duration: float, speed: int): 
        self.moving_straight(duration, speed)
        time.sleep(0.5)
        self.moving_straight(duration, speed)
        time.sleep(0.5)

    def turn_back_slowly(self): 
        self.turn(1, 5)
        self.turn(1, 5)

    #########################################

    def expressJoy(self): 
        self.emotionalmotor_proxy.expressJoy()
        pixel_array = {i: ifaces.RoboCompLEDArray.Pixel(green=170, red=0, blue=85, white=0) for i in range(self.NUM_LEDS)}
        self.ledarray_proxy.setLEDArray(pixel_array)

        self.differentialrobot_proxy.setSpeedBase(5000, 0)
        time.sleep(0.5)
        self.differentialrobot_proxy.setSpeedBase(-5000, 0)
        time.sleep(0.5)
        self.moving_side_to_side(1)
        
    def expressSadness(self): 
        self.emotionalmotor_proxy.expressSadness()
        pixel_array = {i: ifaces.RoboCompLEDArray.Pixel(red=0, green=85, blue=153, white=0) for i in range(self.NUM_LEDS)}
        self.ledarray_proxy.setLEDArray(pixel_array)

        time.sleep(0.5)
        self.moving_straight(1.5, -5) # moving slowly
        time.sleep(0.5)
        self.turn_back_slowly()
        time.sleep(1)
        self.turn_back_slowly()

    def expressFear(self): 
        self.emotionalmotor_proxy.expressFear()
        pixel_array = {i: ifaces.RoboCompLEDArray.Pixel(red=50, green=0, blue=80, white=0) for i in range(self.NUM_LEDS)}
        self.ledarray_proxy.setLEDArray(pixel_array)
        time.sleep(0.5)
        self.jolts(0.1, -100)

    def expressSurprise(self): 
        self.emotionalmotor_proxy.expressSurprise()
        pixel_array = {i: ifaces.RoboCompLEDArray.Pixel(red=255, green=255, blue=102, white=0) for i in range(self.NUM_LEDS)}
        self.ledarray_proxy.setLEDArray(pixel_array)

        self.differentialrobot_proxy.setSpeedBase(-5000, 0) # go back quickly
        time.sleep(0.5)
        
        self.turn_full()
    
    def expressAnger(self): 
        self.emotionalmotor_proxy.expressAnger()
        pixel_array = {i: ifaces.RoboCompLEDArray.Pixel(red=128, green=0, blue=0, white=0) for i in range(self.NUM_LEDS)}
        self.ledarray_proxy.setLEDArray(pixel_array)
        self.jolts(0.1, 100)

    def expressDisgust(self): 
        self.emotionalmotor_proxy.expressDisgust()
        pixel_array = {i: ifaces.RoboCompLEDArray.Pixel(red=50, green=30, blue=10, white=0) for i in range(self.NUM_LEDS)}
        self.ledarray_proxy.setLEDArray(pixel_array)
        
        self.moving_straight(0.5, -1000) # go back
        time.sleep(0.5)

        # simulating saying "no"
        self.turn_left()
        self.turn_right()
        self.turn_right()
        self.turn_left()

        

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


