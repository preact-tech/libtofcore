import pytofcore

def make_vsm():
    new_elements = []
    max_size = 16
    for i in range(max_size):
        e = pytofcore.VsmElement()
        e.modulation_frequency = 12010
        if i % 2:
            e.integration_time = 507
        else:
            e.integration_time = 609
        new_elements.append(e)

    vsm = pytofcore.VsmControl()
    vsm.elements = new_elements
    return vsm


def print_vsm(vsm):
    counter = 0
    for e in vsm.elements:
        print(f"Idx: {counter}, mod_freq: {e.modulation_frequency}")
        print(f"Idx: {counter}, int_time: {e.integration_time}")
        counter += 1

print('initializing sensor')
s = pytofcore.Sensor('/dev/ttyACM0')
s.stop_stream()
info = info = s.get_sensor_info()
print('finished initializing sensor')
print("SENSOR INFO")
print(info)

print("QUERYING VSM")
vsm = s.get_vsm()

print("CURRENT VSM")
print_vsm(vsm)
print("SETTING VSM")
vsm_new = make_vsm()
s.set_vsm(vsm_new)
print("QUERYING NEW VSM")
vsm = s.get_vsm()
print_vsm(vsm)