import argparse
import sys
import pytofcore
from ipaddress import IPv4Interface, IPv4Address

def print_ipv4_settings(sensor:pytofcore.Sensor):
    ipv4Settings = sensor.ipv4_settings
    print(ipv4Settings)
    

def cleanup_and_exit(sensor:pytofcore.Sensor):
    sensor = None
    sys.exit(0)

parser = argparse.ArgumentParser()
parser.add_argument('--protocol-version', default=pytofcore.Sensor.DEFAULT_PROTOCOL_VERSION, type=int, help="Protocol version to use 0 or 1")
parser.add_argument('--port-name', default=pytofcore.Sensor.DEFAULT_PORT_NAME, type=str, help="Port name to connect to sensor on.")
parser.add_argument('ipv4_interface', nargs='?', type=str, default=None, help="Set interface IPv4 address and mask (e.g '10.10.31.80/24`)")
parser.add_argument('ipv4_gateway', nargs='?', type=str, default=[10, 10, 31, 1], help="Set IPv4 Gateway")
args = parser.parse_args()

sensor = pytofcore.Sensor(protocol_version=args.protocol_version, port_name=args.port_name)

if args.ipv4_interface is not None:
    if args.ipv4_interface and not args.ipv4_gateway:
        print("ERROR: gateway setting must be provided when setting interface parameters.")
        exit(0)

    print("Setting IPv4 interface to: ", args.ipv4_interface, ", Gateway: ", args.ipv4_gateway)
    sensor.ipv4_settings = pytofcore.IPv4Settings(IPv4Interface(args.ipv4_interface), IPv4Address(args.ipv4_gateway))

print_ipv4_settings(sensor)

cleanup_and_exit(sensor)