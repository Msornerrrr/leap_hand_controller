#!/usr/bin/env python3
"""Change Dynamixel motor baudrate for all 16 LEAP hand motors.

Usage:
  python3 set_baudrate.py --from 4000000 --to 2000000
  python3 set_baudrate.py --from 2000000 --to 4000000
  python3 set_baudrate.py --from 4000000 --to 2000000 --port /dev/ttyUSB0
"""
import argparse
from dynamixel_sdk import *

# Dynamixel baud rate register value mapping
BAUD_TABLE = {
    9600: 0,
    57600: 1,
    115200: 2,
    1000000: 3,
    2000000: 3,
    3000000: 4,
    4000000: 5,
    4500000: 6,
}
# XC330 uses different values: 0=9600, 1=57600, 2=115200, 3=1M, 4=2M, 5=3M, 6=4M, 7=4.5M
XC330_BAUD_TABLE = {
    9600: 0,
    57600: 1,
    115200: 2,
    1000000: 3,
    2000000: 4,
    3000000: 5,
    4000000: 6,
    4500000: 7,
}

ADDR_TORQUE_ENABLE = 64
ADDR_BAUD_RATE = 8


def main():
    parser = argparse.ArgumentParser(description='Set LEAP hand motor baudrate')
    parser.add_argument('--from', dest='from_baud', type=int, required=True,
                        help='Current baudrate (e.g. 4000000)')
    parser.add_argument('--to', dest='to_baud', type=int, required=True,
                        help='Target baudrate (e.g. 2000000)')
    parser.add_argument('--port', default='/dev/ttyUSB0',
                        help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--ids', type=int, nargs='+', default=list(range(16)),
                        help='Motor IDs (default: 0-15)')
    args = parser.parse_args()

    if args.to_baud not in XC330_BAUD_TABLE:
        print(f"ERROR: Target baudrate {args.to_baud} not supported.")
        print(f"Supported: {list(XC330_BAUD_TABLE.keys())}")
        return

    baud_value = XC330_BAUD_TABLE[args.to_baud]

    port = PortHandler(args.port)
    if not port.openPort():
        print(f"ERROR: Cannot open {args.port}")
        return
    if not port.setBaudRate(args.from_baud):
        print(f"ERROR: Cannot set baudrate {args.from_baud}")
        port.closePort()
        return

    pkt = PacketHandler(2.0)
    print(f"Connected at {args.from_baud} baud on {args.port}")
    print(f"Setting motors to {args.to_baud} baud (register value={baud_value})...\n")

    for mid in args.ids:
        # Disable torque (required before changing baud)
        pkt.write1ByteTxRx(port, mid, ADDR_TORQUE_ENABLE, 0)
        # Set baud rate
        result, err = pkt.write1ByteTxRx(port, mid, ADDR_BAUD_RATE, baud_value)
        if result == COMM_SUCCESS:
            print(f"  Motor {mid:2d}: OK")
        else:
            print(f"  Motor {mid:2d}: FAILED ({pkt.getTxRxResult(result)})")

    port.closePort()
    print(f"\nDone. All motors set to {args.to_baud} baud.")
    print(f"Power cycle the motors, then connect at {args.to_baud}.")


if __name__ == '__main__':
    main()
