import argparse
import cv2 as cv
import numpy as np
import sys
import time

import pytofcrust

sensor = pytofcrust.Sensor(protocol_version=1)
v_flip_default = False
h_flip_default = False


def print_sensor_info():
    sensor_info = sensor.get_sensor_info()
    chip_id = hex(sensor_info.chipId)

    print(f"Sensor Info: \n\
Device Serial # : {sensor_info.deviceSerialNumber}\n\
Cpu Board Serial # : {sensor_info.cpuBoardSerialNumber}\n\
Illuminator Board Serial # : {sensor_info.illuminatorBoardSerialNumber}\n\
Model Name : {sensor_info.modelName}\n\
Last Reset Type : {sensor_info.lastResetType}\n\
Software Version: {sensor_info.softwareVersion}\n\
Cpu Board Version: {sensor_info.cpuVersion}\n\
Chip ID: {chip_id}\n\
Illuminator Board SW Version: {sensor_info.illuminatorSwVersion}.{sensor_info.illuminatorSwId}\n\
Backpack Module Info: {sensor_info.backpackModule}\n")

def measurement_callback(data):
    if data.data_type == pytofcrust.Measurement.DataType.DISTANCE_AMPLITUDE:
        amp = np.array(data.amplitude_data)
        amp_scaled = (amp - amp.min()) / (amp.max() - amp.min())
        amp_uint8 = (255 * amp_scaled).astype(np.uint8)

        cv.imshow('Amplitude - Distance Amplitude', amp_uint8)
        measurement_callback.SWITCH = True
    elif data.data_type == pytofcrust.Measurement.DataType.DCS:
        dcs = np.array(data.dcs_data).astype(np.float64)
        amp = np.sqrt(
            (dcs[0,...]-dcs[2,...]).astype(np.float64)**2 + \
            (dcs[1,...]-dcs[3,...]).astype(np.float64)**2
        )

        amp_scaled = (amp - amp.min()) / (amp.max() - amp.min())
        amp_uint8 = (255 * amp_scaled).astype(np.uint8)
        cv.imshow('Amplitude - DCS Ambient', amp_uint8)
        measurement_callback.SWITCH = True
    if args.show_illmn_info:
        illmn_info = data.illuminator_info
        print(illmn_info)
    if cv.waitKey(1)&0xFF == ord('q'):
        measurement_callback.EXIT = True

def cleanup_and_exit(s:pytofcrust.Sensor):
    s.stop_stream()
    s = None
    cv.destroyAllWindows()
    sys.exit(0)

# Necessary due to an unkown bug in the command protocol
def cmd_delay():
    time.sleep(0.1)

# Setup arg parser and run the tool
parser = argparse.ArgumentParser()
parser.add_argument("-H", "--hFlip", help="Set Horizontal flip state", type=int, default=h_flip_default)
parser.add_argument("-V", "--vFlip", help="Set Vertical flip state", type=int, default=v_flip_default)
parser.add_argument("-s", "--cpu_serial", help="Set CPU board Serial", type=str, default=None)
parser.add_argument("-S", "--dev_serial", help="Set Device serial", type=str, default=None)
parser.add_argument("-i", "--illmn_serial", help="Set Illuminator serial", type=int, default=0)
parser.add_argument("-I", "--show_illmn_info", default=False, action='store_true', help="Show the illuminator info for every frame.")
args = parser.parse_args()

if(args.cpu_serial):
    sensor.set_factory_mode(True)
    if(sensor.set_cpu_board_serial(args.cpu_serial)):
        print(f"Set CPU Board serial to: {args.cpu_serial}")
    else:
        print("Failed to set cpu board serial!")

if(args.dev_serial):
    sensor.set_factory_mode(True)
    if(sensor.set_device_serial(args.dev_serial)):
        print(f"Set device serial to: {args.dev_serial}")
    else:
        print("Failed to set device serial!")

if(args.illmn_serial):
    if(sensor.set_illuminator_serial(args.illmn_serial)):
        print(f"Set Illuminator Board serial to: {args.illmn_serial}")
    else:
        print("Failed to set Illuminator Board serial!")


print_sensor_info()

print("Horizontal Flip Selected: {:d}".format(args.hFlip))
print("Vertical Flip Selected: {:d}".format(args.vFlip))

sensor.hflip = args.hFlip
cmd_delay()
sensor.vflip = args.vFlip
cmd_delay()

print("Horizontal Flip Actual: {}".format(sensor.hflip))
cmd_delay()
print("Vertical Flip Actual: {}".format(sensor.vflip))
cmd_delay()

measurement_callback.EXIT = False
measurement_callback.SWITCH = False

success = sensor.set_integration_times(100,0,0)
cmd_delay()
print(sensor.get_sensor_info)
cmd_delay()
sensor.subscribe_measurement(measurement_callback)

stream_state = 0
stream_modes = (sensor.stream_distance_amplitude, sensor.stream_dcs_ambient)
stream_modes[stream_state]()

try:
    while not measurement_callback.EXIT:
        time.sleep(0.01)
        if measurement_callback.SWITCH:
            stream_state = 1 - stream_state
            stream_modes[stream_state]()
            measurement_callback.SWITCH = False
except KeyboardInterrupt:
    # exit cleanly on ctrl+c
    cleanup_and_exit(sensor)

cleanup_and_exit(sensor)