#!/usr/bin/env python

'''
drop package at specified coordinates
'''

import sys, struct, time, os
import RPi.GPIO as GPIO

from pymavlink import mavutil
from argparse import ArgumentParser

def initLeds():
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for gpionr in ledMap.itervalues():
                GPIO.setup(gpionr, GPIO.OUT)
                GPIO.output(gpionr, GPIO.LOW)

#activates a led and disactivates the others
def activateLed(argLedGpio):
        for gpionr in ledMap.itervalues():
                GPIO.output(gpionr, GPIO.LOW)
        GPIO.output(argLedGpio, GPIO.HIGH)

def disactivateLeds():
        for gpionr in ledMap.itervalues():
                GPIO.output(gpionr, GPIO.LOW)

def enumerateLeds():
        for gpionr in ledMap.itervalues():
                activateLed(gpionr)
                time.sleep(.3)
	disactivateLeds()

def printMissions():
	writeToFile("", fileOut)
        writeToFile("Missions", fileOut)
        writeToFile("--------", fileOut)
        #waypoint read protocol
        master.waypoint_request_list_send()
        msgMissionCount = master.recv_match(type = 'MISSION_COUNT', blocking = True)
        writeToFile("mission count = {0}".format(msgMissionCount.count), fileOut)
        for i in range ( 0, msgMissionCount.count ):
                master.waypoint_request_send(i)
                iterMission = master.recv_match(type = 'MISSION_ITEM', blocking = True)
                writeToFile("mission {0} : ".format(i), fileOut)
                writeToFile(iterMission, fileOut)
        writeToFile("", fileOut)

def writeToFile(argString, argFile):
	if argString:
		print(argString)
	        argFile.write("{0} : {1}\n".format(time.strftime("%Y/%m/%d %H:%M:%S", time.localtime()), argString))
	else:
		argFile.write("\n")
		print("")

parser = ArgumentParser(description=__doc__)

parser.add_argument("--baudrate", type=int, help="master port baud rate", default=115200)
parser.add_argument("--device", required=True, help="serial device")
parser.add_argument("--rate", default=4, type=int, help="requested stream rate")
parser.add_argument("--source-system", dest='SOURCE_SYSTEM', type=int, default=255, help='MAVLink source system for this GCS')
parser.add_argument("--latitude", type=float, help="latitude (float) of package to be dropped", required=True)
parser.add_argument("--longitude", type=float, help="longitude (float) of package to be dropped", required=True)
parser.add_argument("--flyAlt", type=int, help="altitude (int) in meters to fly to", required=True)
parser.add_argument("--dropAlt", type=int, help="altitude (int) in meters of package to be dropped", required=True)
parser.add_argument("--pctForceGripperLeft", type=int, help="pct of the force that has to be used on gripper left when closed", default=100)
parser.add_argument("--pctForceGripperRight", type=int, help="pct of the force that has to be used on gripper left when closed", default=100)

args = parser.parse_args()

#constants init
ledMap = dict(blue = 7, red = 8, green = 24, yellow = 25)
servoMinValue = 500
servoMaxValue = 2300

#generate output filename from scriptname and time
fileOut = open("log_{0}_{1}_{2}.txt".format(str(os.path.splitext(__file__)[0]), time.strftime("%Y%m%d", time.localtime()), time.strftime("%H%M%S", time.localtime())), "w")

initLeds()
enumerateLeds()

writeToFile("Program start", fileOut)

# create a mavlink serial instance
master = mavutil.mavlink_connection(args.device, baud=args.baudrate)

# wait for the heartbeat msg to find the system ID
master.wait_heartbeat()
writeToFile(str("Heartbeat from APM (system {0} component {1})".format(master.target_system, master.target_component)), fileOut)

writeToFile("Sending all stream request for rate {0}".format(args.rate), fileOut)
for i in range(0, 3) :
	master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, args.rate, 1)

writeToFile("wait for gps fix to acquire current location ...", fileOut)
master.wait_gps_fix()
currentLoc = master.location()
writeToFile(currentLoc, fileOut)
#currentLoc = mavutil.location(lat = 50.891215, lng = 3.499862)

printMissions()

writeToFile("clearing missions", fileOut)
master.waypoint_clear_all_send()

printMissions()

#waypoint write protocol : send count; as much mission items demanded as count; mission ack receive
writeToFile("populating mission items", fileOut)
master.mav.mission_count_send(master.target_system, master.target_component, 10)
msgMissionRequest = master.recv_match(type = "MISSION_REQUEST", blocking = True)
writeToFile("mission request 0 from copter", fileOut)
writeToFile(msgMissionRequest, fileOut)
master.mav.mission_item_send(master.target_system, master.target_component, 0, 0, 16, 0, 1, 0, 0, 0, 0, currentLoc.lat, currentLoc.lng, 0)
msgMissionRequest = master.recv_match(type = "MISSION_REQUEST", blocking = True)
writeToFile("mission request 1 from copter", fileOut)
writeToFile(msgMissionRequest, fileOut)
#MAV_CMD_NAV_TAKEOFF to fly altitude
master.mav.mission_item_send(master.target_system, master.target_component, 1, 3, 22, 0, 1, 0, 0, 0, 0, 0, 0, args.flyAlt)
msgMissionRequest = master.recv_match(type = "MISSION_REQUEST", blocking = True)
writeToFile("mission request 2 from copter", fileOut)
writeToFile(msgMissionRequest, fileOut)
#MAV_CMD_NAV_WAYPOINT
master.mav.mission_item_send(master.target_system, master.target_component, 2, 3, 16, 0, 1, 0, 0, 0, 0, args.latitude, args.longitude, args.dropAlt)
msgMissionRequest = master.recv_match(type = "MISSION_REQUEST", blocking = True)
writeToFile("mission request 3 from copter", fileOut)
writeToFile(msgMissionRequest, fileOut)
#MAV_CMD_DO_SET_SERVO for releasing the package
master.mav.mission_item_send(master.target_system, master.target_component, 3, 0, 183, 0, 1, 10, servoMinValue, 0, 0, 0, 0, 0)
msgMissionRequest = master.recv_match(type = "MISSION_REQUEST", blocking = True)
writeToFile("mission request 4 from copter", fileOut)
writeToFile(msgMissionRequest, fileOut)
#MAV_CMD_DO_SET_SERVO for releasing the package
master.mav.mission_item_send(master.target_system, master.target_component, 4, 0, 183, 0, 1, 9, servoMinValue, 0, 0, 0, 0, 0)
msgMissionRequest = master.recv_match(type = "MISSION_REQUEST", blocking = True)
writeToFile("mission request 5 from copter", fileOut)
writeToFile(msgMissionRequest, fileOut)
#MAV_CMD_CONDITION_DELAY to give the package time to drop
master.mav.mission_item_send(master.target_system, master.target_component, 5, 0, 112, 0, 1, 5, 0, 0, 0, 0, 0, 0)
msgMissionRequest = master.recv_match(type = "MISSION_REQUEST", blocking = True)
writeToFile("mission request 6 from copter", fileOut)
writeToFile(msgMissionRequest, fileOut)
#MAV_CMD_DO_SET_SERVO for releasing the package
master.mav.mission_item_send(master.target_system, master.target_component, 6, 0, 183, 0, 1, 10, servoMaxValue, 0, 0, 0, 0, 0)
msgMissionRequest = master.recv_match(type = "MISSION_REQUEST", blocking = True)
writeToFile("mission request 7 from copter", fileOut)
writeToFile(msgMissionRequest, fileOut)
#MAV_CMD_DO_SET_SERVO for releasing the package
master.mav.mission_item_send(master.target_system, master.target_component, 7, 0, 183, 0, 1, 9, servoMaxValue, 0, 0, 0, 0, 0)
msgMissionRequest = master.recv_match(type = "MISSION_REQUEST", blocking = True)
writeToFile("mission request 8 from copter", fileOut)
writeToFile(msgMissionRequest, fileOut)
#MAV_CMD_NAV_LOITER_TIME for 10 sec at drop coordinates (args)
master.mav.mission_item_send(master.target_system, master.target_component, 8, 3, 19, 0, 1, 10, 0, 0, 0, args.latitude, args.longitude, args.dropAlt)
msgMissionRequest = master.recv_match(type = "MISSION_REQUEST", blocking = True)
writeToFile("mission request 9 from copter", fileOut)
writeToFile(msgMissionRequest, fileOut)
#MAV_CMD_NAV_RETURN_TO_LAUNCH
master.mav.mission_item_send(master.target_system, master.target_component, 9, 3, 20, 0, 1, 0, 0, 0, 0, 0, 0, 0)
msgMissionAck = master.recv_match(type = "MISSION_ACK", blocking = True)
writeToFile("mission ack copter", fileOut)
writeToFile(msgMissionAck, fileOut)

printMissions()

#keep track of mission status
currentMission = -1

while True:
	msg = master.recv_match(blocking=True, type='MISSION_CURRENT')
	if(currentMission != msg.seq):
		writeToFile("Mission item {} started".format(msg.seq), fileOut)
		currentMission = msg.seq
		if(msg.seq == 0):
	                activateLed(ledMap["green"])
        	elif(msg.seq == 1):
                	activateLed(ledMap["yellow"])
	        elif(msg.seq == 2):
        	        activateLed(ledMap["blue"])
	        elif(msg.seq == 3):
        	        activateLed(ledMap["red"])
		elif(msg.seq == 4):
                        activateLed(ledMap["green"])
                elif(msg.seq == 5):
                        activateLed(ledMap["yellow"])
                elif(msg.seq == 6):
                        activateLed(ledMap["blue"])
                elif(msg.seq == 7):
                        activateLed(ledMap["red"])
		elif(msg.seq == 8):
                        activateLed(ledMap["green"])
                elif(msg.seq == 9):
                        activateLed(ledMap["yellow"])
                elif(msg.seq == 10):
                        activateLed(ledMap["blue"])
                elif(msg.seq == 11):
                        activateLed(ledMap["red"])
		elif(msg.seq == 12):
                        activateLed(ledMap["green"])
                elif(msg.seq == 13):
                        activateLed(ledMap["yellow"])
                elif(msg.seq == 14):
                        activateLed(ledMap["blue"])
                elif(msg.seq == 15):
                        activateLed(ledMap["red"])
		else:
			disactivateLeds()

#todo : get the rtl height param to navigate to drop dest instead of having to specify it as a command arg

