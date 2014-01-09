HLU
===

HIGH LEVEL UNIT
---------------

HLU folder contains the High Level software:

* **RoboController-PID_Regulator**: Qt application used to tune the PIDs on the RoboController V2 board
* **RoboControllerSDK**: is the robot Qt based SDK, contains all the functions to control a Robot based on RoboController V2 board
* **RoboControllerServer**: is the TCP server that runs an an embedded board suited on the robot and connected to the RoboController V2 board through a serial cable
* **RobotGUI**: is a simple Qt GUI Application to control the robot. It runs on PC and mobile devices
* **common**: contains libraries used by different softwares
