cmake_minimum_required(VERSION 2.6)
PROJECT(  EBOMoods )

INCLUDE( /home/robocomp/robocomp/cmake/robocomp.cmake )

ROBOCOMP_IDSL_TO_ICE( CommonBehavior DifferentialRobot EmergencyStop EmotionalMotor GenericBase LEDArray Speech )
