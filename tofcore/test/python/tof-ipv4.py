import argparse
import sys
import pytofcore
import time
from ipaddress import IPv4Interface, IPv4Address

def print_ipv4_settings(sensor:pytofcore.Sensor):
    ipv4Settings = sensor.ipv4_settings
    print(ipv4Settings)
    logIPv4Settings = sensor.ipv4_log_settings
    print(logIPv4Settings)
    

def cleanup_and_exit(sensor:pytofcore.Sensor):
    sensor = None
    sys.exit(0)

parser = argparse.ArgumentParser()
parser.add_argument('--port-name', default=pytofcore.Sensor.DEFAULT_PORT_NAME, type=str, help="Port name to connect to sensor on.")
parser.add_argument('--ipv4_interface', type=str, default=None, help="Set interface IPv4 address and mask (e.g. '10.10.31.80/24`)")
parser.add_argument('--ipv4_gateway', type=str, default=[10, 10, 31, 1], help="Set IPv4 Gateway (e.g., '10.10.31.1')")
parser.add_argument('--log_dest_addr', type=str, help="Set IPv4 Address for sensor's log output (e.g., '10.10.31.100')")
parser.add_argument('--log_dest_port', type=int, default=5001, help="Set UDP port for sensor's log output (e.g., 5001)")
parser.add_argument('--persist', type=bool, default=False, help="Write UDP destination settings to sensor's persistent storage")
args = parser.parse_args()

sensor = pytofcore.Sensor(port_name=args.port_name)

if args.ipv4_interface is not None:
    if args.ipv4_interface and not args.ipv4_gateway:
        print("ERROR: gateway setting must be provided when setting interface parameters.")
        exit(0)

    print("Setting IPv4 interface to: ", args.ipv4_interface, ", Gateway: ", args.ipv4_gateway)
    sensor.ipv4_settings = pytofcore.IPv4Settings(IPv4Interface(args.ipv4_interface), IPv4Address(args.ipv4_gateway))
    time.sleep(1.0)

if args.log_dest_addr is not None:

    print("Setting sensor log destination to: ", args.log_dest_addr, ":", args.log_dest_port)
    sensor.ipv4_log_settings = pytofcore.IPv4LogSettings(IPv4Address(args.log_dest_addr), args.log_dest_port)
    time.sleep(1.0)
    
if args.persist:
    
    print("Updating sensor's persistent settings")
    sensor.storeSettings()
    time.sleep(1.0)

print_ipv4_settings(sensor)

cleanup_and_exit(sensor)
