
from typing import List
import time
from pytofcore import VsmControl, VsmElement
import pytofcore

def set_vsm(sensor, integration_time_list: List[int]) -> None:
    '''
    Utility function that wraps the sensor.set_vsm command
    '''
    # build vsm control and pass to sensor
    new_elements = [] 
    max_size = len(integration_time_list)
    for i in range(max_size):
        e = VsmElement()
        e.modulation_frequency = 12000
        e.integration_time = integration_time_list[i]
        new_elements.append(e)
    vsm = VsmControl()
    vsm.elements = new_elements
    sensor.set_vsm(vsm)

def measurement_callback(m):
    int_time = m.integration_time
    print(f'received a measurement... int_time: {int_time}')


s = pytofcore.Sensor("10.10.31.180")

info = s.get_sensor_info()

print(info)

s.subscribe_measurement(measurement_callback)
#
# NOTE: The following sequence had problems:
#       1) Start streaming
#       2) Send a command right away
# This loop repeats that sequence to test whether the problem
# has been fixed. When not fixed, the set_vsm() call fails
# (throws an exception) because the sensor didn't see the command
#
while (True):
    s.stream_distance_amplitude()
    #time.sleep(1.0) # NOTE: Enabling this allows the set_vsm to work

    set_vsm(s, [100,1000])
    print("VSM set")

    time.sleep(1.0) # Let it stream for 2 seconds

    s.stop_stream()
