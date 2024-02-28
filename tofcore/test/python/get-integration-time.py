import argparse
import sys

import pytofcore

sensor = pytofcore.Sensor()

def print_integration_time():
    integration_time = sensor.get_integration_time()
    print(integration_time)
    

def cleanup_and_exit(s:pytofcore.Sensor):
    s = None
    sys.exit(0)

# Setup arg parser and run the tool

print_integration_time()

cleanup_and_exit(sensor)
