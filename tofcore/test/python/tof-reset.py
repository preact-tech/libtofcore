import argparse
import sys
import pytofcore

def cleanup_and_exit(sensor:pytofcore.Sensor):
    sensor = None
    sys.exit(0)

parser = argparse.ArgumentParser()
parser.add_argument('--port-name', default=pytofcore.Sensor.DEFAULT_PORT_NAME, type=str, help="Port name to connect to sensor on.")
args = parser.parse_args()

sensor = pytofcore.Sensor(port_name=args.port_name)

sensor.reset_sensor()

cleanup_and_exit(sensor)
