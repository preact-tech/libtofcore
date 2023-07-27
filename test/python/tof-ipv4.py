import argparse
import sys

import pytofcore

sensor = pytofcore.Sensor(protocol_version=1)

def print_ipv4_settings():
    ipv4Settings = sensor.ipv4_settings
    print(ipv4Settings)
    

def cleanup_and_exit(s:pytofcore.Sensor):
    s = None
    sys.exit(0)

# Setup arg parser and run the tool

print_ipv4_settings()

cleanup_and_exit(sensor)