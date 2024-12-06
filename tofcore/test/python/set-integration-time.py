import argparse
import sys
import pytofcore
import time
from lxml.cssselect import ns


parser = argparse.ArgumentParser()
parser.add_argument('--delay', default=0.0, type=float, help="Delay this long (seconds) between set and get")
parser.add_argument('--get', default=False, type=bool, help="Get and Print integration time (uS)")
parser.add_argument('--port', default=pytofcore.Sensor.DEFAULT_PORT_NAME, type=str, help="Port name to connect to sensor on.")
parser.add_argument('--repeat', default=0.0, type=float, help="Repeat at after this delay (seconds) until killed")
parser.add_argument('--set', default=None, type=int, help="Set integration time (uS)")
parser.add_argument('--stats', default=False, type=bool, help="Print stats on command times")
args = parser.parse_args()

repeat_count = 0

def print_integration_time():
    start_ns = time.time_ns()
    integration_time = sensor.get_integration_time()
    end_ns = time.time_ns()
    if args.stats:
        print(repeat_count, ": ", integration_time, " uS (command time: ", ((end_ns - start_ns)/1000000), " mS)")
    else:
        print(repeat_count, ": ", integration_time, " uS")

def cleanup_and_exit(s:pytofcore.Sensor):
    s = None
    sys.exit(0)

sensor = pytofcore.Sensor(port_name=args.port)

while (repeat_count < 1) or (args.repeat != 0.0):

    if args.set is not None:
        start_ns = time.time_ns()
        sensor.set_integration_time(args.set)
        end_ns = time.time_ns()
        if args.stats:
            print("Set command time: ", ((end_ns - start_ns)/1000000), " mS")
            
    if (args.delay != 0.0):
        time.sleep(args.delay)

    if args.get:
        print_integration_time()
        
    repeat_count += 1
    
    time.sleep(args.repeat)

cleanup_and_exit(sensor)
