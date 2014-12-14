#!/usr/bin/env python

'''
set stream rate on an APM
'''

import sys, struct, time, os

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

print("1 wait for servo output msg")
for i in range(0, 10):
    msg = master.recv_match(blocking=True, type='SERVO_OUTPUT_RAW')
    print(msg)

print("set servo 1");
#master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 1, 1200, 0, 0, 0, 0, 0)
master.set_servo(1, 500)

msg = master.recv_match(blocking=True, type='COMMAND_ACK')
print(msg)

#master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, args.rate, 1)

print("2 wait for servo output msg")
for i in range(0, 10):
    msg = master.recv_match(blocking=True, type='SERVO_OUTPUT_RAW')
    print(msg)
