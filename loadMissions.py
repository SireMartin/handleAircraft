#!/usr/bin/env python

'''
set stream rate on an APM
'''

import sys, struct, time, os
import RPi.GPIO as GPIO

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--baudrate", type=int,
                  help="master port baud rate", default=115200)
parser.add_argument("--device", required=True, help="serial device")
parser.add_argument("--rate", default=4, type=int, help="requested stream rate")
parser.add_argument("--source-system", dest='SOURCE_SYSTEM', type=int,
                  default=255, help='MAVLink source system for this GCS')
parser.add_argument("--showmessages", action='store_true',
                  help="show incoming messages", default=False)
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

#enumerateLeds()

# create a mavlink serial instance
master = mavutil.mavlink_connection(args.device, baud=args.baudrate)

# wait for the heartbeat msg to find the system ID
master.wait_heartbeat()
print("Heartbeat from APM (system %u component %u)" % (master.target_system, master.target_component))

print("Sending all stream request for rate %u" % args.rate)
#for i in range(0, 3) :
master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, args.rate, 1)

print("wait for gps fix ...")
master.wait_gps_fix()
currentLoc = master.location()
print("first location ")
print(currentLoc);
#GPIO.output(7, GPIO.HIGH)
#time.sleep(1)
#GPIO.output(7, GPIO.LOW)
#wait 30 sec for another (better) gps fix
#time.sleep(30
#currentLoc = master.location()
#print("second location ")
#print(currentLoc);
#GPIO.output(8, GPIO.HIGH)
#time.sleep(1)
#GPIO.output(8, GPIO.LOW)

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

printMissions()

print("clearing missions")
master.waypoint_clear_all_send()

printMissions()

print("populating mission items")
master.mav.mission_count_send(master.target_system, master.target_component, 3)
msgMissionRequest = master.recv_match(type = "MISSION_REQUEST", blocking = True)
print("mission request 0 from copter")
print(msgMissionRequest)
master.mav.mission_item_send(master.target_system, master.target_component, 0, 0, 16, 0, 1, 0, 0, 0, 0, currentLoc.lat, currentLoc.lng, 0)
#master.mav.mission_item_send(master.target_system, master.target_component, 0, 0, 16, 0, 1, 0, 0, 0, 0, 50.922800, 3.447932, 0)
msgMissionRequest = master.recv_match(type = "MISSION_REQUEST", blocking = True)
print("mission request 1 from copter")
print(msgMissionRequest)
master.mav.mission_item_send(master.target_system, master.target_component, 1, 3, 22, 0, 1, 0, 0, 0, 0, 0, 0, 10)
msgMissionRequest = master.recv_match(type = "MISSION_REQUEST", blocking = True)
print("mission request 2 from copter")
print(msgMissionRequest)
master.mav.mission_item_send(master.target_system, master.target_component, 2, 3, 16, 0, 1, 0, 0, 0, 0, 60, 5, 10)
msgMissionAck = master.recv_match(type = "MISSION_ACK", blocking = True)
print("mission ack copter")
print(msgMissionAck)

#while True :
#	msgMissionRequest = master.recv_match(type = "MISSION_REQUEST", blocking = True)
#	print("mission request 2 from copter")
#	print(msgMissionRequest)

printMissions()
