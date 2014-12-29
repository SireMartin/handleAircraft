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
args = parser.parse_args()

from pymavlink import mavutil

# create a mavlink serial instance
master = mavutil.mavlink_connection(args.device, baud=args.baudrate)

master.wait_heartbeat()
print("Heartbeat from APM (system %u component %u)" % (master.target_system, master.target_component))

#print("Sending all stream request for rate %u" % args.rate)
master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, args.rate, 1)

msg = master.recv_match(type = 'SERVO_OUTPUT_RAW', blocking = True)
print(msg)

master.set_servo(9, 1000)

#while True:
#	msg = master.recv_msg()
#	print(msg)

msg = master.recv_match(type = 'COMMAND_ACK', blocking = True)
print(msg)
