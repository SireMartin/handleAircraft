#!/usr/bin/env python

'''
drop package at specified coordinates
'''

import sys, struct, time, os
import RPi.GPIO as GPIO

from argparse import ArgumentParser
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

from pymavlink import mavutil

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(7, GPIO.OUT)		#blauw
GPIO.setup(8, GPIO.OUT)		#rood 
GPIO.setup(24, GPIO.OUT)	#groen
GPIO.setup(25, GPIO.OUT)	#geel

def enumerateLeds():
	#test leds
	GPIO.output(7, GPIO.HIGH)
	time.sleep(1)
	GPIO.output(7, GPIO.LOW)
	GPIO.output(8, GPIO.HIGH)
	time.sleep(1)
	GPIO.output(8, GPIO.LOW)
	GPIO.output(24, GPIO.HIGH)
	time.sleep(1)
	GPIO.output(24, GPIO.LOW)
	GPIO.output(25, GPIO.HIGH)
	time.sleep(1)
	GPIO.output(25, GPIO.LOW)
	
def printMissions() :
	print("\nMissions")
	print  ("--------")
	#waypoint read protocol
	master.waypoint_request_list_send()
	msgMissionCount = master.recv_match(type = 'MISSION_COUNT', blocking = True)
	print("mission count = {0}".format(msgMissionCount.count))
	for i in range ( 0, msgMissionCount.count ):
        	master.waypoint_request_send(i)
		iterMission = master.recv_match(type = 'MISSION_ITEM', blocking = True)
	        print("mission {0} : ".format(i))
	        print iterMission
	print("")


#enumerateLeds()

# create a mavlink serial instance
master = mavutil.mavlink_connection(args.device, baud=args.baudrate)

# wait for the heartbeat msg to find the system ID
master.wait_heartbeat()
print("Heartbeat from APM (system %u component %u)" % (master.target_system, master.target_component))

print("Sending all stream request for rate %u" % args.rate)
for i in range(0, 3) :
	master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, args.rate, 1)

print("wait for gps fix ...")
master.wait_gps_fix()
currentLoc = master.location()
print("first location ")
print(currentLoc);
#currentLoc = mavutil.location(lat = 50.891215, lng = 3.499862)

printMissions()

print("clearing missions")
master.waypoint_clear_all_send()

printMissions()

#waypoint write protocol : send count; as much mission items demanded as count; mission ack receive
print("populating mission items")
master.mav.mission_count_send(master.target_system, master.target_component, 4)
msgMissionRequest = master.recv_match(type = "MISSION_REQUEST", blocking = True)
print("mission request 0 from copter")
print(msgMissionRequest)
master.mav.mission_item_send(master.target_system, master.target_component, 0, 0, 16, 0, 1, 0, 0, 0, 0, currentLoc.lat, currentLoc.lng, 0)
msgMissionRequest = master.recv_match(type = "MISSION_REQUEST", blocking = True)
print("mission request 1 from copter")
print(msgMissionRequest)
#MAV_CMD_NAV_TAKEOFF to 10 meters
master.mav.mission_item_send(master.target_system, master.target_component, 1, 3, 22, 0, 1, 0, 0, 0, 0, 0, 0, args.flyAlt)
msgMissionRequest = master.recv_match(type = "MISSION_REQUEST", blocking = True)
print("mission request 2 from copter")
print(msgMissionRequest)
#MAV_CMD_NAV_LOITER_TIME for 15 sec at drop coordinates (args)
master.mav.mission_item_send(master.target_system, master.target_component, 2, 3, 19, 0, 1, 15, 0, 0, 0, args.latitude, args.longitude, args.dropAlt)
msgMissionRequest = master.recv_match(type = "MISSION_REQUEST", blocking = True)
print("mission request 3 from copter")
print(msgMissionRequest)
#MAV_CMD_NAV_RETURN_TO_LAUNCH
master.mav.mission_item_send(master.target_system, master.target_component, 3, 3, 20, 0, 1, 0, 0, 0, 0, 0, 0, 0)
msgMissionAck = master.recv_match(type = "MISSION_ACK", blocking = True)
print("mission ack copter")
print(msgMissionAck)

printMissions()

while True:
    msg = master.recv_match(blocking=True, type='MISSION_CURRENT')
    print("Mission item reached with seq %u" % msg.seq)
    if(msg.seq == 0):
        print("seq 0 : steek gele led aan")
        GPIO.output(25, GPIO.HIGH)
    if(msg.seq == 1):
        print("seq 1 : steek groene led aan")
        GPIO.output(24, GPIO.HIGH)
    if(msg.seq == 2):
        print("seq 2 : steek rode led aan")
        GPIO.output(8, GPIO.HIGH)
    if(msg.seq == 3):
        print("seq 3 : steek blauwe led aan") 
        GPIO.output(7, GPIO.HIGH)
    time.sleep(0.1)

#currentTicksGripperLeft/Right
#currentAircraftMode (enum above)



#todo : get the rtl height param to navigate to drop dest instead of having to specify it as a command arg
