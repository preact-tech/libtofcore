import argparse
import sys

import pytofcore

sensor = pytofcore.Sensor(protocol_version=1)

def print_integration_times():
    integration_times = sensor.get_integration_times()
    print(integration_times)
    

def cleanup_and_exit(s:pytofcore.Sensor):
    s = None
    sys.exit(0)

# Setup arg parser and run the tool

print_integration_times()

cleanup_and_exit(sensor)