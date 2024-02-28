import pytofcore

def make_vsm():
    new_elements = []
    max_size = 16
    for i in range(max_size):
        e = pytofcore.VsmElement()
        e.m_modulationFreqKhz = 12010
        if i % 2:
            e.m_integrationTimeUs = 507
        else:
            e.m_integrationTimeUs = 609
        new_elements.append(e)

    vsm = pytofcore.VsmControl()
    vsm.m_numberOfElements = max_size
    vsm.m_elements = new_elements
    return vsm


def print_vsm(vsm):
    counter = 0
    for e in vsm.m_elements:
        print(f"Idx: {counter}, mod_freq: {e.m_integrationTimeUs}")
        print(f"Idx: {counter}, int_time: {e.m_modulationFreqKhz}")
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