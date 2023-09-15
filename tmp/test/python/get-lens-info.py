import argparse
import sys

import pytofcore

sensor = pytofcore.Sensor(protocol_version=1)

def print_lens_info():
    lensInfo = sensor.lens_info
    print(lensInfo)
    

def cleanup_and_exit(s:pytofcore.Sensor):
    s = None
    sys.exit(0)

# Setup arg parser and run the tool

print_lens_info()

cleanup_and_exit(sensor)