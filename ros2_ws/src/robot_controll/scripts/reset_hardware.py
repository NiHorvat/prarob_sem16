#!/usr/bin/env python3
"""
Reboot Dynamixel servos to clear Hardware Error Status flags.

Run this when the launch log shows:
  [FATAL] [DynamixelHardware]: [RxPacketError] Hardware error occurred.

Rebooting clears the error register and re-enables torque so the
hardware plugin can initialise cleanly on the next launch.

Usage:
  python3 reset_hardware.py
  # or after colcon build:
  ros2 run robot_controll reset_hardware.py
"""

import sys
try:
    from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
except ImportError:
    print("ERROR: dynamixel_sdk not found.  Install with:")
    print("  pip install dynamixel_sdk")
    sys.exit(1)

PORT  = '/dev/ttyUSB0'
BAUD  = 1_000_000
IDS   = [12, 13, 11]
PROTO = 2.0

port = PortHandler(PORT)
pkt  = PacketHandler(PROTO)

if not port.openPort():
    print(f"ERROR: cannot open {PORT}")
    sys.exit(1)

port.setBaudRate(BAUD)
print(f"Opened {PORT} at {BAUD} baud")

all_ok = True
for dxl_id in IDS:
    result, error = pkt.reboot(port, dxl_id)
    if result == COMM_SUCCESS:
        print(f"  ID {dxl_id}: rebooted OK (hardware error cleared)")
    else:
        print(f"  ID {dxl_id}: FAILED — {pkt.getTxRxResult(result)}")
        all_ok = False

port.closePort()

if all_ok:
    print("\nAll servos rebooted. Wait ~1 s then re-launch.")
else:
    print("\nSome servos did not respond — check cable / power.")
    sys.exit(1)
