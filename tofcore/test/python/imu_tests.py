import argparse
import pytofcore
import signal
import sys
import time

def signal_handler(sig, frame):
    print('stop requested')
    signal_handler.stop = True

signal_handler.stop = False
signal.signal(signal.SIGINT, signal_handler)

parser = argparse.ArgumentParser(epilog="EXAMPLE: imu_tests.py -p /dev/ttyACM0")
parser.add_argument('-p','--port_name', default=pytofcore.Sensor.DEFAULT_PORT_NAME, type=str, help="Port name for connecting to sensor.")
parser.add_argument('-d','--data', default=False, action='store_true',  help="Returns the IMU sensor data.")
parser.add_argument('-a','--accel_selftest', default=False, action='store_true',  help="Executes IMU accelerometer self-test.")
args = parser.parse_args()

sensor = pytofcore.Sensor(port_name=args.port_name)
info = sensor.get_sensor_info()
print(f"{info}\n")


def cleanup_and_exit(s:pytofcore.Sensor):
    s = None
    sys.exit(0)

try:

    if args.data:
        res = sensor.get_imu_data()
        print(f"Successfully retrieved IMU data with get_imu_data results={res}")

    if args.accel_selftest:
        res = sensor.imu_accelerometer_self_test()
        print(f"Successfully ran imu_accelerometer_self_test results={res}")
    
except Exception as e:
    print(e)

cleanup_and_exit(sensor)
