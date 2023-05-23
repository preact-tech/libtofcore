import pytofcore
import time

# Test case for MOS-426

sensor = pytofcore.Sensor(port_name='/dev/ttyACM0')
print(sensor.get_sensor_info())

for i in range(1,100000+1):
    try:
        sensor.set_integration_time(100,0,0,0)
        # time.sleep(0.001) # This one is usually fine
        # time.sleep(0.0001) # This one occasionally throws errors
        pass # This one pretty much always throws errors and can hang your program
    except Exception as e:
        print(e)
        