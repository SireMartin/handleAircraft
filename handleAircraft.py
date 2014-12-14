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

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_component))

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(7, GPIO.OUT)		#blauw
GPIO.setup(8, GPIO.OUT)		#rood 
GPIO.setup(24, GPIO.OUT)	#groen
GPIO.setup(25, GPIO.OUT)	#geel

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

# create a mavlink serial instance
master = mavutil.mavlink_connection(args.device, baud=args.baudrate)

# wait for the heartbeat msg to find the system ID
#wait_heartbeat(master)
master.wait_heartbeat()
print("Heartbeat from APM (system %u component %u)" % (master.target_system, master.target_component))

print("Sending all stream request for rate %u" % args.rate)
master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, args.rate, 1)

print("request mission list")
master.mav.mission_request_list_send(master.target_system, master.target_component)

print("wait for the mission_count msg")
msg = master.recv_match(blocking=True, type='MISSION_COUNT')
print(msg)

print("current (system %u component %u)" % (master.mav.srcSystem, master.mav.srcComponent))

qtyMissionItems = msg.count
print("Mission items count = %u" % qtyMissionItems)

for i in range(0, qtyMissionItems):
    master.mav.mission_request_send(master.target_system, master.target_component, i)
    msg = master.recv_match(blocking=True, type='MISSION_ITEM')
    '''print("mission item %u :" % i)'''
    print(msg)

master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, args.rate, 1)

print("wachten op gps fix...")
master.wait_gps_fix()
#when there is a fix, we can get a location
currentLoc = master.location()
print("gps fixed op home location = ")
print(currentLoc)

GPIO.output(7, GPIO.HIGH)
time.sleep(1)
GPIO.output(7, GPIO.LOW)

#eerst moet de safety knop ingerukt worden. let op want de eerste keer in de keuken is er bloed gevallen
#master.arducopter_arm()

master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, args.rate, 1)

#open file to log iten reached messages to
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
