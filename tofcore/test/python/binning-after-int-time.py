#! /usr/bin/env python3

from time import sleep

import cv2 as cv
import numpy as np
import time

import pytofcore

sensor = pytofcore.Sensor()

def measurement_callback(data):
    if data.data_type == pytofcore.Measurement.DataType.DISTANCE_AMPLITUDE:
        amp = np.array(data.amplitude_data)
        amp_scaled = (amp - amp.min()) / (amp.max() - amp.min())
        amp_uint8 = (255 * amp_scaled).astype(np.uint8)

        cv.imshow('Amplitude - Distance Amplitude', amp_uint8)
        measurement_callback.SWITCH = True
    elif data.data_type == pytofcore.Measurement.DataType.DCS:
        dcs = np.array(data.dcs_data).astype(np.float64)
        amp = np.sqrt(
            (dcs[0,...]-dcs[2,...]).astype(np.float64)**2 + \
            (dcs[1,...]-dcs[3,...]).astype(np.float64)**2
        )

        amp_scaled = (amp - amp.min()) / (amp.max() - amp.min())
        amp_uint8 = (255 * amp_scaled).astype(np.uint8)
        cv.imshow('Amplitude - DCS Ambient', amp_uint8)
        measurement_callback.SWITCH = True
    if cv.waitKey(1)&0xFF == ord('q'):
        measurement_callback.EXIT = True

measurement_callback.EXIT = False
measurement_callback.SWITCH = False

success = sensor.set_integration_time(100)
#sleep(0.1)
sensor.set_binning(vertical=False, horizontal=False)
print(sensor.get_sensor_info)
sensor.subscribe_measurement(measurement_callback)

stream_state = 0
stream_modes = (sensor.stream_distance_amplitude, sensor.stream_dcs_ambient)
stream_modes[stream_state]()

while not measurement_callback.EXIT:
    time.sleep(0.01)
    if measurement_callback.SWITCH:
        stream_state = 1 - stream_state
        stream_modes[stream_state]()
        measurement_callback.SWITCH = False

sensor.stop_stream()
cv.destroyAllWindows()