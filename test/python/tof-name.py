import argparse
import sys
import pytofcore

def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def print_name(sensor:pytofcore.Sensor):
    print("Name:", sensor.sensor_name)

def cleanup_and_exit(sensor:pytofcore.Sensor):
    sensor = None
    sys.exit(0)

parser = argparse.ArgumentParser()
parser.add_argument('--protocol-version', default=pytofcore.Sensor.DEFAULT_PROTOCOL_VERSION, type=int, help="Protocol version to use 0 or 1")
parser.add_argument('--port-name', default=pytofcore.Sensor.DEFAULT_PORT_NAME, type=str, help="Port name to connect to sensor on.")
parser.add_argument('--sensor-name', type=str, default=None, help="Set sensor name")
parser.add_argument('--store-settings', type=str2bool, nargs='?', const=True, default=False, help="Store the sensor settings in persistent memory")
args = parser.parse_args()

sensor = pytofcore.Sensor(protocol_version=args.protocol_version, port_name=args.port_name)

if args.sensor_name is not None:
    print("Setting sensor name to:", args.sensor_name)
    sensor.sensor_name = args.sensor_name
    
if args.store_settings:
    print("Storing sensor settings to persistent memory")
    sensor.storeSettings()

print_name(sensor)

cleanup_and_exit(sensor)