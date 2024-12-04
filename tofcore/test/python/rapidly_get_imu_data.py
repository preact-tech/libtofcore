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

cmd_count = 0
frame_count = 0
cum_cmd_duration = 0.0;
max_cmd_duration = 0.0;
min_cmd_duration = 60.0;

parser = argparse.ArgumentParser(epilog="EXAMPLE: rapidly_get_imu_data.py -p /dev/ttyACM0 -t 0.04 -T 60")
parser.add_argument('-p','--port_name', default=pytofcore.Sensor.DEFAULT_PORT_NAME, type=str, help="Port name for connecting to sensor.")
parser.add_argument('-a','--test_dcs_ambient', default=False, action='store_true',  help="Test while streaming DCS+AMBIENT.")
parser.add_argument('-d','--test_distance_amplitude', default=False, action='store_true', help="Test while streaming DISTANCE+AMPLITUDE.")
parser.add_argument('-D','--test_dcs_diff_ambient', default=True, action='store_false', help="Do NOT test while streaming DCS_DIFF+AMBIENT.")
parser.add_argument('-t','--time_btw_commands', type=float, default=0.05, help="Time (sec) between commands.")
parser.add_argument('-T','--test_time', type=float, default=10.0, help="Duration (sec) of test loop.")
args = parser.parse_args()

sensor = pytofcore.Sensor(port_name=args.port_name)

info = sensor.get_sensor_info()
print(f"{info}\n")

def callback(m):
    global frame_count
    frame_count += 1

def cleanup_and_exit(s:pytofcore.Sensor):
    sensor.stop_stream()
    s = None
    sys.exit(0)

sensor.subscribe_measurement(callback)

def query_imu():
    global cmd_count
    global cum_cmd_duration
    global max_cmd_duration
    global min_cmd_duration
    last_cmd_start_time = time.time()
    now = last_cmd_start_time
    end_time = now + args.test_time
    while now < end_time:
        now = time.time()
        wait_time = (last_cmd_start_time + args.time_btw_commands) - now
        if wait_time > 0.0:
            time.sleep(wait_time)
        last_cmd_start_time = time.time()
        before = last_cmd_start_time
        
        res = sensor.get_imu_data()
        
        after = time.time()
        cmd_count += 1
        cmd_duration = after - before
        if cmd_duration > max_cmd_duration:
            max_cmd_duration = cmd_duration
        if cmd_duration < min_cmd_duration:
            min_cmd_duration = cmd_duration
        cum_cmd_duration += cmd_duration
        if cmd_duration > 1.5:
            print(f"Getting IMU data took {cmd_duration}")
            raise RuntimeError("Setting integration time took > 1.5 seconds...")

try:

    if args.test_dcs_ambient:
        print("Entering distance amplitude mode")
        sensor.stream_distance_amplitude()
        query_imu()
        print("Successfully queried IMU data while in distance amplitude mode\n")

    if args.test_distance_amplitude:
        print("Entering DCS + Ambient mode")
        sensor.stream_dcs_ambient()
        query_imu()
        print("Successfully queried IMU data while in dcs ambient mode\n")

    if args.test_dcs_diff_ambient:
        print("Entering DCS_DIFF + Ambient mode")
        sensor.stream_dcs_diff_ambient()
        query_imu()
        print("Successfully queried IMU data while in dcs diff + ambient mode\n")

    print(f'PASSED. Received {frame_count} frames. Got IMU data {cmd_count} times.')
    print(f'Command times (avg/min/max): {cum_cmd_duration/cmd_count} / {min_cmd_duration} / {max_cmd_duration}')
    
except Exception as e:
    
    print(e)
    print (f'FAILED. Received {frame_count} frames. Got IMU data {cmd_count} times.')

cleanup_and_exit(sensor)
