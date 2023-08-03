import argparse
import sys
import pytofcore

def print_name(sensor:pytofcore.Sensor):
    print("Name:", sensor.sensor_name)

def cleanup_and_exit(sensor:pytofcore.Sensor):
    sensor = None
    sys.exit(0)

parser = argparse.ArgumentParser()
parser.add_argument('--protocol-version', default=pytofcore.Sensor.DEFAULT_PROTOCOL_VERSION, type=int, help="Protocol version to use 0 or 1")
parser.add_argument('--port-name', default=pytofcore.Sensor.DEFAULT_PORT_NAME, type=str, help="Port name to connect to sensor on.")
parser.add_argument('--sensor-name', type=str, default=None, help="Set sensor name")
args = parser.parse_args()

sensor = pytofcore.Sensor(protocol_version=args.protocol_version, port_name=args.port_name)

if args.sensor_name is not None:
    print("Setting sensor name to:", args.sensor_name)
    sensor.sensor_name = args.sensor_name

print_name(sensor)

cleanup_and_exit(sensor)