import argparse
import sys
import pytofcore

def print_ipv4_settings(sensor:pytofcore.Sensor):
    ipv4Settings = sensor.ipv4_settings
    print(ipv4Settings)
    

def cleanup_and_exit(sensor:pytofcore.Sensor):
    sensor = None
    sys.exit(0)

parser = argparse.ArgumentParser()
parser.add_argument('--protocol-version', default=pytofcore.Sensor.DEFAULT_PROTOCOL_VERSION, type=int, help="Protocol version to use 0 or 1")
parser.add_argument('--port-name', default=pytofcore.Sensor.DEFAULT_PORT_NAME, type=str, help="Port name to connect to sensor on.")
parser.add_argument('-a', '--ipv4_addr', nargs="+", type=int, default=None, help="Set IPv4 Address")
parser.add_argument('-m', '--ipv4_mask', nargs="+", type=int, default=[255, 255, 255, 0], help="Set IPv4 Mask")
parser.add_argument('-g', '--ipv4_gateway', nargs="+", type=int, default=[10, 10, 31, 1], help="Set IPv4 Gateway")
args = parser.parse_args()

sensor = pytofcore.Sensor(protocol_version=args.protocol_version, port_name=args.port_name)

if args.ipv4_addr is not None:
    print("Setting IPv4Addr: ", args.ipv4_addr, " Mask: ", args.ipv4_mask, " Gateway: ", args.ipv4_gateway)
    sensor.set_ipv4(args.ipv4_addr, args.ipv4_mask, args.ipv4_gateway)

print_ipv4_settings(sensor)

cleanup_and_exit(sensor)