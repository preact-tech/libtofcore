import pytofcore
import numpy as np
import time

# Find and setup our sensor
ports = pytofcore.find_all_devices()
if len(ports) == 0:
    print("No sensor found")
    exit()
port_name = ports[0].connector_uri  # By default taking the first one.

sensor = pytofcore.Sensor(port_name=port_name)

#Use streaming mechanism and a counter as a poor mans method
# of collecting only 1 measurement of each type of frame.
# in the future we can have a blocking call that returns a single measurement
measurements = []

def example_callback(data):
    if example_callback.counter < 2: # Gather 2 frames of data
        measurements.append(data)
    example_callback.counter += 1
example_callback.counter = 0

# Add the example call back function to when the sensor takes a measurement 
sensor.subscribe_measurement(example_callback)

# Start streaming
sensor.stream_distance_amplitude()

# Continue stream while collecting
while example_callback.counter < 1:
    time.sleep(0.1)
sensor.stop_stream()

#Sleep a little to allow any pending frames to flush out of the queue. 
time.sleep(0.5)

sensor = None

#Print out the data in measurements
for data in measurements:
    print(f"Measurement type: {data.data_type}")
    if data.data_type == pytofcore.Measurement.DataType.DISTANCE_AMPLITUDE:
        distance_data = data.distance_data
        amplitude_data = data.amplitude_data
        distance = np.array(distance_data)
        print(f"  distance: shape {distance.shape}, ndim {distance.ndim}, type.name {distance.dtype.name}, itemsize {distance.itemsize}, size {distance.size}")
        print(distance)
        amplitude = np.array(amplitude_data)
        print(f"  amplitude: shape {amplitude.shape}, ndim {amplitude.ndim}, type.name {amplitude.dtype.name}, itemsize {amplitude.itemsize}, size {amplitude.size}")
        print(amplitude)