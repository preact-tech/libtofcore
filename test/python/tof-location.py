import argparse
import sys
import pytofcore

def print_location(sensor:pytofcore.Sensor):
    print("Location:", sensor.sensor_location)

def cleanup_and_exit(sensor:pytofcore.Sensor):
    sensor = None
    sys.exit(0)

parser = argparse.ArgumentParser()
parser.add_argument('--protocol-version', default=pytofcore.Sensor.DEFAULT_PROTOCOL_VERSION, type=int, help="Protocol version to use 0 or 1")
parser.add_argument('--port-name', default=pytofcore.Sensor.DEFAULT_PORT_NAME, type=str, help="Port name to connect to sensor on.")
parser.add_argument('--sensor-location', type=str, default=None, help="Set sensor location")
args = parser.parse_args()

sensor = pytofcore.Sensor(protocol_version=args.protocol_version, port_name=args.port_name)

if args.sensor_location is not None:
    print("Setting sensor location to:", args.sensor_location)
    sensor.sensor_location = args.sensor_location

print_location(sensor)

cleanup_and_exit(sensor)