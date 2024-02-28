import pytofcore
import numpy as np
import time
import argparse


parser = argparse.ArgumentParser()
parser.add_argument('--port-name', default='/dev/ttyACM1', type=str, help="Serial (type) device to connect to sensor", )
parser.add_argument('--integration', default=1, type=int, help="Integration time to run at", )
parser.add_argument('--plot-dcs-histogram', action='store_true', help="use matplotlib to plot historgram of dcs values" )
args = parser.parse_args()

#Note: until we get auto discovery working you may need to specify the
#      ttyACM port for the device if it's not ttyACM0.
#      e.g. sensor = pytofcore.Sensor(port_name="/dev/ttyACM1")
sensor = pytofcore.Sensor(port_name = args.port_name)

#Use streaming mechanism and a counter as a poor mans method
# of collecting only 1 measurement of each type of frame.
# in the future we can have a blocking call that returns a single measurement
measurements = []

def example_callback(data):
    if example_callback.counter < 2:
        measurements.append(data)
    example_callback.counter += 1


sensor.subscribe_measurement(example_callback)

#stream_dcs_ambient() results in 2 measurement types being streamed:
# - DCS measurement with 4 DCS frames included
# - Grayscale measurement with single grayscale frame
example_callback.counter = 0
sensor.stream_dcs_ambient()
while example_callback.counter < 1:
    time.sleep(0.1)
sensor.stop_stream()

#Sleep a little to allow any pending DCS frames to flush out of the queue. 
# TODO: Need a better way to deal with this
time.sleep(0.5)

#Stream_distance_amplitude results in 1 measurement type which includes
# both the distance and amplitude frame
example_callback.counter = 1
sensor.stream_distance_amplitude()
while example_callback.counter < 2:
    time.sleep(0.1)
sensor.stop_stream()

sensor = None

#Note: The *_data attributes for the Measurement class return MemoryView's which provide
#      transparent fast access to the raw memory buffers owned by the Measurement instances.
#      MemoryViews are **NOT** managed by Python, the underlying memory is only valid so long
#      as the original Measurement instance continues to exist. Attempting to store and access
#      after it's Measurement instance has been destroyed will result in undefined behavior.
for data in measurements:
    print(f"Measurement type: {data.data_type}")
    if data.data_type == pytofcore.Measurement.DataType.DCS:
        dcs_data = data.dcs_data
        dcs = np.array(dcs_data)
        print(f"  shape {dcs.shape}, ndim {dcs.ndim}, type.name {dcs.dtype.name}, itemsize {dcs.itemsize}, size {dcs.size}")
        if args.plot_dcs_histogram:
            import matplotlib.pyplot as plt
            plt.hist(dcs.ravel(), bins=100)
            plt.grid()
            plt.title('Histogram of DCS Values')
            plt.show()
    if data.data_type == pytofcore.Measurement.DataType.GRAYSCALE:
        grayscale = np.array(data.grayscale_data)
        print(f"  shape {grayscale.shape}, ndim {grayscale.ndim}, type.name {grayscale.dtype.name}, itemsize {grayscale.itemsize}, size {grayscale.size}")
    if data.data_type == pytofcore.Measurement.DataType.DISTANCE_AMPLITUDE:
        distance_data = data.distance_data
        amplitude_data = data.amplitude_data
        distance = np.array(distance_data)
        print(f"  distance: shape {distance.shape}, ndim {distance.ndim}, type.name {distance.dtype.name}, itemsize {distance.itemsize}, size {distance.size}")
        amplitude = np.array(amplitude_data)
        print(f"  amplitude: shape {amplitude.shape}, ndim {amplitude.ndim}, type.name {amplitude.dtype.name}, itemsize {amplitude.itemsize}, size {amplitude.size}")






