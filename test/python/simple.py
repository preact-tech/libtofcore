#For this import the pytofcore.*.so module file must be someplace in the PYTHONPATH
# this can be done by adding the build directory to PYTHONPATH (e.g. `export PTYHONPATH=$PYTHONPATH:<path-to-build-dir`)
# or by installing the library to your python site-packages directory. 

import argparse
import signal
import pytofcore
import time
import timeit

def signal_handler(sig, frame):
    print('stop requested')
    signal_handler.stop = True

signal_handler.stop = False
signal.signal(signal.SIGINT, signal_handler)

parser = argparse.ArgumentParser()
parser.add_argument('--protocol-version', default=pytofcore.Sensor.DEFAULT_PROTOCOL_VERSION, type=int, help="Protocol version to use 0 or 1")
parser.add_argument('--port-name', default=pytofcore.Sensor.DEFAULT_PORT_NAME, type=str, help="Port name to connect to sensor on.")
args = parser.parse_args()

s = pytofcore.Sensor(protocol_version=args.protocol_version, port_name=args.port_name)

def callback(frame, **kwargs) :
    '''Function to be called when a new frame of data is received
       Note this callback will be called from background thread that is
       managed by the Sensor object.
       Do not attempt modify the sensor from this callback or you will deadlock!
    '''
    callback.counter += 1

    if callback.counter == 1:
        print(f"First frame received size: {len(frame.dcs_data)}")
    if callback.counter == 5:
        #Exceptions raised from the callback are sent to stderr
        raise RuntimeError("Wow something really bad happend")

    if callback.counter % 10 == 0:
        print("Hey look I've received 10 more frames: ", callback.counter)

callback.counter = 0

s.set_integration_time(1, 0, 0)
s.subscribe_measurement(callback)
s.stream_dcs()
t_0 = timeit.default_timer()

while (not signal_handler.stop) and (callback.counter < 50):
    time.sleep(1)

t_1 = timeit.default_timer()
s.stop_stream()

elapsed_time = (t_1 - t_0)
print(f"received {callback.counter} frames in {elapsed_time} s")
print(f"frame rate {callback.counter/elapsed_time} fps")
