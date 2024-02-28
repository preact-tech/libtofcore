import pytofcore
import numpy as np
import time
import argparse


parser = argparse.ArgumentParser()
parser.add_argument('--port-name', default='/dev/ttyACM0', type=str, help="Serial (type) device to connect to sensor", )
parser.add_argument('--integration', default=1, type=int, help="Integration time to run at", )
parser.add_argument('--plot-ambient-histogram', action='store_true', help="use matplotlib to plot historgram of ambient values" )
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
    if example_callback.counter < 4:
        measurements.append(data)
    example_callback.counter += 1


sensor.subscribe_measurement(example_callback)

#stream_dcs_ambient() results in 2 measurement types being streamed:
# - DCS measurement with 4 DCS frames included
# - Grayscale measurement with single ambient frame
example_callback.counter = 0
sensor.set_integration_time(args.integration)
sensor.stream_dcs_ambient()
while example_callback.counter < 1:
    time.sleep(0.1)
sensor.stop_stream()

#Sleep a little to allow any pending DCS frames to flush out of the queue. 
# TODO: Need a better way to deal with this
time.sleep(0.5)

sensor = None

#Note: The *_data attributes for the Measurement class return MemoryView's which provide
#      transparent fast access to the raw memory buffers owned by the Measurement instances.
#      MemoryViews are **NOT** managed by Python, the underlying memory is only valid so long
#      as the original Measurement instance continues to exist. Attempting to store and access
#      after it's Measurement instance has been destroyed will result in undefined behavior.
for data in measurements:
    print(f"Measurement type: {data.data_type}")
    if data.data_type == pytofcore.Measurement.DataType.AMBIENT:
        ambient_data = data.ambient_data
        ambient = np.array(ambient_data)
        print(f"  shape {ambient.shape}, ndim {ambient.ndim}, type.name {ambient.dtype.name}, itemsize {ambient.itemsize}, size {ambient.size}")
        if args.plot_ambient_histogram:
            import matplotlib.pyplot as plt
            plt.hist(ambient.ravel(), bins=100)
            plt.grid()
            plt.title('Histogram of Ambient Values')
            plt.show()






