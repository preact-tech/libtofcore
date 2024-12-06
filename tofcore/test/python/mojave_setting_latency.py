import pytofcore
import time
import queue

global_queue = queue.Queue()


# callback for receiving data
def measurement_callback(data):
    print('receiving data...')
    # get the current request info
    if not global_queue.empty():
        requested_int_time, need_new_request, requested_timestamp = global_queue.get()
        # check if new data matches request
        if data.integration_time == requested_int_time:
            # new data matches request, report latency and create a new request
            now = time.time()
            latency = now - requested_timestamp
            print(f"Set integration time latency: {latency}")
            requested_int_time += 1
            need_new_request = True
            global_queue.put((requested_int_time, need_new_request, 0.0))
        else:
            global_queue.put((requested_int_time, need_new_request, requested_timestamp))


# initialize sensor
s = pytofcore.Sensor()
s.subscribe_measurement(measurement_callback)
s.stop_stream()
info = s.get_sensor_info()
s.stream_distance_amplitude()
print(info)

# initial request
global_queue.put((100, True, time.time()))
time.sleep(0.1)
#s.stream_distance_amplitude()
while True:
    if not global_queue.empty():
        requested_int_time, need_new_request, requested_timestamp = global_queue.get()
        if need_new_request:
            # need to request new integration time from sensor
            requested_timestamp = time.time()
            need_new_request = False
            global_queue.put((requested_int_time, need_new_request, requested_timestamp))
            stamp = time.time()
            s.stop_stream()
            s.set_integration_time(requested_int_time)
            s.stream_distance_amplitude()
#            s.stream_dcs()
            set_elapsed_time = time.time() - stamp
            print(f'set_integration_time blocking time: {set_elapsed_time}')
        else:
            global_queue.put((requested_int_time, need_new_request, requested_timestamp))

    time.sleep(0.05)
