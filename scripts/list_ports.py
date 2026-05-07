"""
scripts/list_ports.py
=====================
Lists all available serial ports on the system.
Use this to find the correct port for --port (e.g. COM3, /dev/ttyACM0).
"""

import sys
try:
    import serial.tools.list_ports
except ImportError:
    print("❌ Error: 'pyserial' not installed. Run: pip install pyserial")
    sys.exit(1)

def list_serial_ports():
    try:
        ports = serial.tools.list_ports.comports()
    except Exception as e:
        print(f"Error listing ports: {e}")
        return

    if not ports:
        print("No serial ports found. Is the robot plugged in?")
        return

    print(f"Found {len(ports)} serial port(s):")
    print("-" * 50)
    for port, desc, hwid in sorted(ports):
        print(f"Port: {port}")
        print(f"   Description: {desc}")
        print(f"   HWID: {hwid}")
        print("-" * 50)


if __name__ == "__main__":
    list_serial_ports()
