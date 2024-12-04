#!/bin/python3

import argparse
import signal
import sys

import pytofcore
from scipy.stats._mstats_basic import mode

def signal_handler(sig, frame):
    print('stop requested')
    signal_handler.stop = True

signal_handler.stop = False
signal.signal(signal.SIGINT, signal_handler)

parser = argparse.ArgumentParser(epilog="EXAMPLE: get-limits.py -p /dev/ttyACM0")
parser.add_argument('-p','--port_name', default=pytofcore.Sensor.DEFAULT_PORT_NAME, type=str, help="Port name for connecting to sensor.")
args = parser.parse_args()

sensor = pytofcore.Sensor(port_name=args.port_name)

def print_frame_period_and_limits():
    frame_period_data = sensor.get_frame_period_and_limits()
    frame_period, minimum_frame_period, maximum_frame_period = frame_period_data
    print(
        f"Frame Period\n"
        f"\tcurrent:\t{frame_period} mS\n"
        f"\tminimum:\t{minimum_frame_period} mS\n"
        f"\tmaximum:\t{maximum_frame_period} mS")

def print_integration_time_and_limits():
    integration_time_data = sensor.get_integration_time_and_limits()
    integration_time, minimum_integration_time, maximum_integration_time = integration_time_data
    print(
        f"Integration Time\n"
        f"\tcurrent:\t{integration_time} uS\n"
        f"\tminimum:\t{minimum_integration_time} uS\n"
        f"\tmaximum:\t{maximum_integration_time} uS")

def print_modulation_frequency_and_limits():
    modulation_freq_data = sensor.get_modfreq_and_limits_and_step()
    modulation_freq, minimum_modulation_freq, maximum_modulation_freq, modulation_freq_step = modulation_freq_data
    print(
        f"Modulation Frequency\n"
        f"\tcurrent:\t{modulation_freq} kHz\n"
        f"\tminimum:\t{minimum_modulation_freq} kHz\n"
        f"\tmaximum:\t{maximum_modulation_freq} kHz\n"
        f"\tstep size:\t{modulation_freq_step} kHz")

def print_max_vsm_elements():
    max_vsm_elements = sensor.get_vsm_max_number_of_elements()
    print(
        f"VSM\n"
        f"\tMaximum # of elements:\t{max_vsm_elements}")

def print_min_amplitude_and_limits():
    min_amplitude_data = sensor.get_min_amplitude_and_limits()
    min_amplitude, minimum_min_amplitude, maximum_min_amplitude = min_amplitude_data
    print(
        f"Minimum Amplitude\n"
        f"\tcurrent:\t{min_amplitude}\n"
        f"\tminimum:\t{minimum_min_amplitude}\n"
        f"\tmaximum:\t{maximum_min_amplitude}")
    
def print_vled_setting_and_limits():
    vled_limit_data = sensor.get_vled_setting_and_limits()
    vled_setting, vled_min, vled_max = vled_limit_data
    print(
        f"Vled Setting\n"
        f"\tcurrent:\t{vled_setting}\n"
        f"\tminimum:\t{vled_min}\n"
        f"\tmaximum:\t{vled_max}")

def cleanup_and_exit(s:pytofcore.Sensor):
    s = None
    sys.exit(0)

print_frame_period_and_limits()
print_integration_time_and_limits()
print_modulation_frequency_and_limits()
print_max_vsm_elements()
print_min_amplitude_and_limits()
print_vled_setting_and_limits()

cleanup_and_exit(sensor)
