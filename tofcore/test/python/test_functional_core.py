import time
import struct
import pytest
import pytofcore
import threading
import random
import ipaddress
from typing import List
from functools import partial
from ipaddress import IPv4Interface, IPv4Address

def pytest_configure(config):
    config.addinivalue_line(
        "markers", "functional: mark test to run only on named environment"
    )

@pytest.fixture(scope="function")
def dut(request) -> pytofcore.Sensor:
    args = {}

    if request.config.getoption('--sensor-uri'):
        args['uri'] = request.config.getoption('--sensor-uri')

    sensor = pytofcore.Sensor(**args)
    yield sensor
    sensor.stop_stream()
    sensor = None


@pytest.mark.functional
def test_get_sensor_info(dut: pytofcore.Sensor):
    
    versionInfo = dut.get_sensor_info()

    print("\n")
    print("Device Serial number: " + versionInfo.deviceSerialNumber)
    print("CPU Board Serial number: " + versionInfo.cpuBoardSerialNumber)
    print("Illuminator Serial Number: " + versionInfo.illuminatorBoardSerialNumber)
    print("Model name: " + versionInfo.modelName)
    print("Last Reset Type: " + versionInfo.lastResetType)

    print("Software Source ID: " + versionInfo.softwareId)
    print("Sensor Description: " + versionInfo.softwareVersion)

    print("CPU Board Hardware Version: " + str(versionInfo.cpuVersion))
    print("Preact Chip ID: " + hex(versionInfo.chipId))
    print("Illuminator Version: " + versionInfo.illuminatorSwVersion + "." + versionInfo.illuminatorSwId)
    print("Backpack Module/Version: " + str(versionInfo.backpackModule))

    assert versionInfo._fields == ('deviceSerialNumber',
                                   'cpuBoardSerialNumber',
                                   'illuminatorBoardSerialNumber',
                                   'modelName',
                                   'lastResetType',
                                   'softwareId',
                                   'softwareVersion',
                                   'cpuVersion',
                                   'chipId',
                                   'illuminatorSwVersion',
                                   'illuminatorSwId',
                                   'illuminatorHwCfg',
                                   'backpackModule')


@pytest.mark.functional
def test_sensor_udp_log_dest(dut: pytofcore.Sensor):
    
    original_udp_dest = dut.ipv4_log_settings
    
    test_port = 1234
    test_adrs = '10.10.31.123'
    dut.ipv4_log_settings = pytofcore.IPv4LogSettings(IPv4Address(test_adrs), test_port)

    udp_dest = dut.ipv4_log_settings
    assert udp_dest.port == test_port
    assert str(udp_dest.address) == test_adrs

    test_port = 4321
    test_adrs = '10.10.31.231'
    dut.ipv4_log_settings = pytofcore.IPv4LogSettings(IPv4Address(test_adrs), test_port)

    udp_dest = dut.ipv4_log_settings
    assert udp_dest.port == test_port
    assert str(udp_dest.address) == test_adrs
    
    # Retore original settings 
    dut.ipv4_log_settings = pytofcore.IPv4LogSettings(original_udp_dest.address, original_udp_dest.port)

    udp_dest = dut.ipv4_log_settings
    assert udp_dest.port == original_udp_dest.port
    assert udp_dest.address == original_udp_dest.address


@pytest.mark.functional
def test_min_amplitude(dut: pytofcore.Sensor):
    
    original_amplitude_data = dut.get_min_amplitude()
    original_amplitude = original_amplitude_data
    
    dut.set_min_amplitude(original_amplitude + 42)
    amplitude = dut.get_min_amplitude()
    assert amplitude is not None, "No minAmplitude"
    assert amplitude == (original_amplitude + 42), "minAmplitude not correct"
    
    dut.set_min_amplitude(original_amplitude)
    amplitude = dut.get_min_amplitude()
    assert amplitude is not None, "No minAmplitude"
    assert amplitude == original_amplitude, "minAmplitude not correct"
   

@pytest.mark.functional
def test_get_frame_period_limits(dut: pytofcore.Sensor):
    
    frame_period_limits_data = dut.get_frame_period_and_limits()
    frame_period, minimum_frame_period, maximum_frame_period = frame_period_limits_data
 
    assert frame_period_limits_data is not None, "No frame limit data received"
    assert frame_period is not None, "No frame period received"
    assert minimum_frame_period is not None, "No frame min received"
    assert maximum_frame_period is not None, "No frame max received"

    print("\n")
    print("Frame period: " + str(frame_period) + " mS")
    print("Frame period MIN: " + str(minimum_frame_period) + " mS")
    print("Frame period MAX: " + str(maximum_frame_period) + " mS")

@pytest.mark.functional
def test_get_imu_data(dut: pytofcore.Sensor):
    
    imu_data = dut.get_imu_data()

    print("\n")
    print("Imu data \n")
    print("Accelerometer milli-g (x,y,z): " + str(imu_data.accel_millig[0]) + " " + str(imu_data.accel_millig[1]) + " " + str(imu_data.accel_millig[2]))
    print("Gyro milli-deg/sec (x,y,z): " + str(imu_data.gyro_milliDegreesPerSecond[0]) + " " + str(imu_data.gyro_milliDegreesPerSecond[1]) + " " + str(imu_data.gyro_milliDegreesPerSecond[2]))
    print("Temperature milli-degC: " + str(imu_data.temperature_milliDegreesC))
    print("Timestamp ms: " + str(imu_data.timestamp))

    assert imu_data._fields == ('accel_millig', 'gyro_milliDegreesPerSecond', 'temperature_milliDegreesC', 'timestamp')

@pytest.mark.functional
def test_get_integration_time_limits(dut: pytofcore.Sensor):
    
    integ_time_limits_data = dut.get_integration_time_and_limits()
    integ_time, minimum_integ_time, maximum_integ_time = integ_time_limits_data
 
    assert integ_time_limits_data is not None, "No integration time limit data received"
    assert integ_time is not None, "No integration time received"
    assert minimum_integ_time is not None, "No integration time min received"
    assert maximum_integ_time is not None, "No integration time max received"

    print("\n")
    print("Integration Time: " + str(integ_time) + " mS")
    print("Integration Time MIN: " + str(minimum_integ_time) + " mS")
    print("Integration Time MAX: " + str(maximum_integ_time) + " mS")


@pytest.mark.functional
def test_get_modulation_frequency_limits(dut: pytofcore.Sensor):
    
    mod_freq_limits_data = dut.get_modfreq_and_limits_and_step()
    mod_freq, minimum_mod_freq, maximum_mod_freq, mod_freq_step = mod_freq_limits_data
 
    assert mod_freq_limits_data is not None, "No modulation frequency limit data received"
    assert mod_freq is not None, "No modulation frequency received"
    assert minimum_mod_freq is not None, "No modulation frequency min received"
    assert maximum_mod_freq is not None, "No modulation frequency max received"
    assert mod_freq_step is not None, "No modulation frequency max received"

    print("\n")
    print("modulation frequency: " + str(mod_freq) + " mS")
    print("modulation frequency MIN: " + str(minimum_mod_freq) + " mS")
    print("modulation frequency MAX: " + str(maximum_mod_freq) + " mS")
    print("modulation frequency STEP: " + str(mod_freq_step) + " mS")


@pytest.mark.functional
def test_get_min_amplitude_limits(dut: pytofcore.Sensor):
    
    min_amplitude_limits_data = dut.get_min_amplitude_and_limits()
    min_amplitude, minimum_min_amplitude, maximum_min_amplitude = min_amplitude_limits_data
 
    assert min_amplitude_limits_data is not None, "No modulation frequency limit data received"
    assert min_amplitude is not None, "No modulation frequency received"
    assert minimum_min_amplitude is not None, "No modulation frequency min received"
    assert maximum_min_amplitude is not None, "No modulation frequency max received"

    print("\n")
    print("minAmplitude: " + str(min_amplitude) + " mS")
    print("minAmplitude MIN: " + str(minimum_min_amplitude) + " mS")
    print("minAmplitude MAX: " + str(maximum_min_amplitude) + " mS")

@pytest.mark.functional
def test_hdr_settings(dut: pytofcore.Sensor):
    def run(settings: List[bool]):
        dut.set_hdr(settings[0], settings[1])
        new_settings = dut.get_hdr_settings()
        assert new_settings.enabled == settings[0], "HDR enabled state mismatch"
        if settings[1]:
            assert new_settings.mode == pytofcore.HdrModeEnum.SPATIAL, "Incorrect HDR Mode"
        else:
            assert new_settings.mode == pytofcore.HdrModeEnum.TEMPORAL, "Incorrect HDR Mode"

    hdr_settings = [[False, False], [True, False], [False, True], [True, True]]
    for setting in hdr_settings:
        run(setting)


@pytest.mark.functional
def test_stream_distance_amplitude_frames(dut: pytofcore.Sensor):
    def callback(measurement: pytofcore.Measurement, **kwargs):
        with callback.mutex:
            if not callback.measurement and (measurement.data_type == pytofcore.Measurement.DataType.DISTANCE_AMPLITUDE):
                callback.measurement = measurement

    callback.mutex = threading.Lock()
    callback.measurement = None
    dut.subscribe_measurement(callback)
    dut.stream_distance_amplitude()
    # It can take several hundred mS for the sensor to initially calculate
    # the correction values for modulation frequencies that are not exactly
    # the calibration frequencies (it has to interpolate)
    count = 0 # we will wait for upto 1 second in 0.1 second increments
    while count != 10:
        count += 1
        time.sleep(0.1)
        with callback.mutex:
            if callback.measurement:
                count = 10
    dut.stop_stream()
    assert callback.measurement is not None, "No Distance & Amplitude measurements received"


@pytest.mark.functional
def test_stream_distance(dut: pytofcore.Sensor):

    def callback(measurement: pytofcore.Measurement, **kwargs):
        with callback.mutex:
            if not callback.measurement and (measurement.data_type == pytofcore.Measurement.DataType.DISTANCE):
                callback.measurement = measurement

    callback.mutex = threading.Lock()
    callback.measurement = None
    dut.subscribe_measurement(callback)
    dut.stream_distance()
    # It can take several hundred mS for the sensor to initially calculate
    # the correction values for modulation frequencies that are not exactly
    # the calibration frequencies (it has to interpolate)
    count = 0 # we will wait for upto 1 second in 0.1 second increments
    while count != 10:
        count += 1
        time.sleep(0.1)
        with callback.mutex:
            if callback.measurement:
                count = 10
    dut.stop_stream()
    assert callback.measurement is not None, "No Distance measurements received"


@pytest.mark.functional
def test_stream_dcs(dut: pytofcore.Sensor):

    def callback(measurement: pytofcore.Measurement, **kwargs):
        with callback.mutex:
            if not callback.measurement and (measurement.data_type == pytofcore.Measurement.DataType.DCS):
                callback.measurement = measurement

    callback.mutex = threading.Lock()
    callback.measurement = None
    dut.subscribe_measurement(callback)
    dut.stream_dcs()
    # It can take several hundred mS for the sensor to initially calculate
    # the correction values for modulation frequencies that are not exactly
    # the calibration frequencies (it has to interpolate)
    count = 0 # we will wait for upto 1 second in 0.1 second increments
    while count != 10:
        count += 1
        time.sleep(0.1)
        with callback.mutex:
            if callback.measurement:
                count = 10
    dut.stop_stream()
    assert callback.measurement is not None, "No DCS measurements received"


@pytest.mark.functional
def test_stream_dcs_ambient(dut: pytofcore.Sensor):
    
    def callback(measurement: pytofcore.Measurement, **kwargs):
        with callback.mutex:
            if not callback.dcs_measurement and (measurement.data_type == pytofcore.Measurement.DataType.DCS):
                callback.dcs_measurement = measurement
            if not callback.ambient_measurement and (measurement.data_type == pytofcore.Measurement.DataType.AMBIENT):
                callback.ambient_measurement = measurement

    callback.mutex = threading.Lock()
    callback.dcs_measurement = None
    callback.ambient_measurement = None
    dut.subscribe_measurement(callback)
    dut.stream_dcs_ambient()
    # It can take several hundred mS for the sensor to initially calculate
    # the correction values for modulation frequencies that are not exactly
    # the calibration frequencies (it has to interpolate)
    count = 0 # we will wait for upto 1 second in 0.1 second increments
    while count != 10:
        count += 1
        time.sleep(0.1)
        with callback.mutex:
            if callback.dcs_measurement and callback.ambient_measurement:
                count = 10
    dut.stop_stream()
    assert callback.dcs_measurement is not None, "No DCS measurements received"
    assert callback.ambient_measurement is not None, "No ambient measurements received"


@pytest.mark.functional
def test_stream_dcs_diff_ambient(dut: pytofcore.Sensor):
    
    def callback(measurement: pytofcore.Measurement, **kwargs):
        with callback.mutex:
            if not callback.dcs_diff_measurement and (measurement.data_type == pytofcore.Measurement.DataType.DCS_DIFF_AMBIENT):
                callback.dcs_diff_measurement = measurement

    callback.mutex = threading.Lock()
    callback.dcs_diff_measurement = None
    dut.subscribe_measurement(callback)
    dut.stream_dcs_diff_ambient()
    # It can take several hundred mS for the sensor to initially calculate
    # the correction values for modulation frequencies that are not exactly
    # the calibration frequencies (it has to interpolate)
    count = 0 # we will wait for upto 1 second in 0.1 second increments
    while count != 10:
        count += 1
        time.sleep(0.1)
        with callback.mutex:
            if callback.dcs_diff_measurement:
                count = 10
    dut.stop_stream()
    assert callback.dcs_diff_measurement is not None, "No DCS_DIFF_AMBIENT measurements received"


@pytest.mark.functional
def test_get_lens_rays(dut: pytofcore.Sensor):

    pixel_rays = dut.pixel_rays
    
    assert pixel_rays._fields == ('x', 'y', 'z')
    assert isinstance(pixel_rays.x, list)
    assert isinstance(pixel_rays.y, list)
    assert isinstance(pixel_rays.z, list)
    assert len(pixel_rays.x) == (320*240)
    assert len(pixel_rays.y) == (320*240)
    assert len(pixel_rays.z) == (320*240)


@pytest.mark.functional
def test_ipv4_settings(dut: pytofcore.Sensor):

    settings = dut.ipv4_settings
    assert( isinstance(settings, pytofcore.IPv4Settings))
    assert( isinstance(settings.interface, ipaddress.IPv4Interface))
    assert( isinstance(settings.interface, ipaddress.IPv4Address))

    with pytest.raises(TypeError, match="pytofcore.IPv4Settings"):
        dut.ipv4_settings = None

    with pytest.raises(TypeError, match="ipaddress.IPv4Interface"):
        dut.ipv4_settings = pytofcore.IPv4Settings(None, None)

    with pytest.raises(TypeError, match="ipaddress.IPv4Address"):
        dut.ipv4_settings = pytofcore.IPv4Settings(settings.interface, None)
    
    dut.ipv4_settings = pytofcore.IPv4Settings(settings.interface, settings.gateway)

    time.sleep(2.0)

    assert settings == dut.ipv4_settings


@pytest.mark.functional
def test_ip_measurement_endpoint(dut: pytofcore.Sensor):
    '''Test the pytofcore.Sensor.ip_measurement_endpoint property

    Verify that the measurement endpoint property can be properly set and retrived.
    That the property only accepts a valid endpoint (which is tuple of IP address and port). 
    '''

    ep = dut.ip_measurement_endpoint
    assert( isinstance(ep, pytofcore.IPv4Endpoint))
    assert( isinstance(ep.address, ipaddress.IPv4Address))
    assert( isinstance(ep.port, int))

    with pytest.raises(TypeError, match="pytofcore.IPv4Endpoint"):
        dut.ip_measurement_endpoint = None

    with pytest.raises(ipaddress.AddressValueError, match=r"Expected 4 octets in 'None'"):
        dut.ip_measurement_endpoint = pytofcore.IPv4Endpoint(None, None)

    with pytest.raises(TypeError, match=r"int\(\) argument must be a string, a bytes-like object or a (?:real\s)*number, not 'NoneType'"):
        dut.ip_measurement_endpoint = pytofcore.IPv4Endpoint(ep.address, None)

    #Verify the measurement_endpoint can be set
    dut.ip_measurement_endpoint = pytofcore.IPv4Endpoint(ep.address, ep.port)
    assert ep == dut.ip_measurement_endpoint

    #Verify the measurement_endpoint can be set using a string for address and port
    dut.ip_measurement_endpoint = pytofcore.IPv4Endpoint(str(ep.address), str(ep.port))
    assert ep == dut.ip_measurement_endpoint


@pytest.mark.functional
def test_meta_data_sensor_temperature(dut: pytofcore.Sensor):
    def callback(measurement: pytofcore.Measurement, **kwargs):
        with callback.mutex:
            if measurement.data_type == measurement.DataType.DISTANCE and not callback.measurement:
                callback.measurement = measurement

    callback.mutex = threading.Lock()
    callback.measurement = None
    dut.subscribe_measurement(callback)
    dut.stream_distance()
    count = 0 # we will wait for upto 1 second in 0.1 second increments
    while count != 10:
        count += 1
        time.sleep(0.1)
        with callback.mutex:
            if callback.measurement:
                count = 10

    dut.stop_stream()
        
    assert callback.measurement, "No measurement received"

    #check for temperature data, note there is no good way to check for actual
    # values so we just check for 4 floats in a reasonable range.
    sensor_temps = callback.measurement.sensor_temperatures
    assert sensor_temps, "No sensor temperature data included with the measurement"
    assert len(sensor_temps) == 4, "Not enough sensor temperature values in the meta-data"
    for v in sensor_temps:
        assert isinstance(v, float), "Sensor temperature value is not a float"
        assert 0 < v < 60.0, "Sensor temperature value appears to be out of range"


@pytest.mark.functional
def test_meta_data_integration_time(dut: pytofcore.Sensor):

    def run(TEST_VALUE):
        def callback(measurement: pytofcore.Measurement, **kwargs):
            with callback.mutex:
                callback.count += 1
                #Note: for some reason it seems to take at least 3 measurement iterations 
                # for new integration time settings to take effect
                # this is probably a bug in the sensor firmware but it's not import ATM
                if callback.count > 3:
                    if measurement.data_type == measurement.DataType.DISTANCE and not callback.measurement:
                        callback.measurement = measurement

        callback.mutex = threading.Lock()
        callback.count = 0
        callback.measurement = None
        dut.set_integration_time(TEST_VALUE)
        dut.subscribe_measurement(callback)
        dut.stream_distance()
        count = 0 # we will wait for upto 1 second in 0.1 second increments
        while count != 10:
            count += 1
            time.sleep(0.1)
            with callback.mutex:
                if callback.measurement:
                    count = 10
        dut.stop_stream()
        
        assert callback.measurement, "No measurement received"

        #check for integration time data with the measurement.
        int_time = callback.measurement.integration_time
        assert int_time, "No integration time data included with the measurement"
        assert TEST_VALUE == int_time, "Incorrect integration time value included in meta-data"
        int_time = dut.get_integration_time()
        assert TEST_VALUE == int_time, "Incorrect integration time value from get_integration_time()"

    run(11)
    run(100)
    run(111)


@pytest.mark.functional
def test_meta_data_modulation_frequency(dut: pytofcore.Sensor):

    def run(freq_khz_to_set: int, freq_khz_expected):
        def callback(measurement: pytofcore.Measurement, **kwargs):
            with callback.mutex:
                callback.count += 1
                #Note: for some reason it seems to take at least 3 measurement iterations 
                # for new integration time settings to take effect
                # this is probably a bug in the sensor firmware but it's not import ATM
                if callback.count > 3:
                    if measurement.data_type == measurement.DataType.DISTANCE and not callback.measurement:
                        callback.measurement = measurement

        callback.mutex = threading.Lock()
        callback.count = 0
        callback.measurement = None

        dut.modulation_frequency = freq_khz_to_set
        dut.subscribe_measurement(callback)
        dut.stream_distance()
        count = 0 # we will wait for upto 1 second in 0.1 second increments
        while count != 20:
            count += 1
            time.sleep(0.1)
            with callback.mutex:
                if callback.measurement:
                    count = 20
        dut.stop_stream()
        
        assert callback.measurement, "No measurement received"

        mod_freq = callback.measurement.modulation_frequency
        assert mod_freq, "No modulation frequency data included with the measurement"
        assert freq_khz_expected == mod_freq/1000, "Incorrect modulation frequency values included in meta-data"

    run(700, 6000)
    run(7500, 7500)
    run(15000, 15000)
    run(30000, 24000)
    run(6000, 6000)
    run(12000, 12000)
    run(24000, 24000)
    run(12001, 12000)
    run(12006, 12010)


@pytest.mark.functional
def test_meta_data_binning(dut: pytofcore.Sensor):

    def run(TEST_VALUE: List[int]):
        def callback(measurement: pytofcore.Measurement, **kwargs):
            with callback.mutex:
                callback.count += 1
                #Note: for some reason it seems to take at least 3 measurement iterations 
                # for new settings to take effect this is probably a bug in the sensor
                # firmware but it's not import ATM
                if callback.count > 3:
                    if measurement.data_type == measurement.DataType.DISTANCE and not callback.measurement:
                        callback.measurement = measurement

        callback.mutex = threading.Lock()
        callback.count = 0
        callback.measurement = None

        h_setting = TEST_VALUE[0] == 2
        v_setting = TEST_VALUE[1] == 2
        dut.set_binning(v_setting, h_setting)
        dut.subscribe_measurement(callback)
        dut.stream_distance()
        count = 0 # we will wait for upto 2 seconds in 0.1 second increments
        while count != 20:
            count += 1
            time.sleep(0.1)
            with callback.mutex:
                if callback.measurement:
                    count = 20
        dut.stop_stream()
        
        assert callback.measurement, "No measurement received"

        vertical_binning = callback.measurement.vertical_binning
        horizontal_binning = callback.measurement.horizontal_binning
        assert vertical_binning is not None, "No vertical binning data included with the measurement"
        assert horizontal_binning is not None, "No horizontal binning data included with the measurement"

        '''
        Binning on individual axes has been deprecated, so setting either H binning or V binning should enable full binning.
        '''
        if 2 in TEST_VALUE:
            assert horizontal_binning == 2, "Incorrect horizontal binning value included in meta-data"
            assert vertical_binning == 2, "Incorrect vertical binning value included in meta-data"
        else:
            assert horizontal_binning == 0, "Incorrect horizontal binning value included in meta-data"
            assert vertical_binning == 0, "Incorrect vertical binning value included in meta-data"

    run([2, 2])
    run([0, 2])
    run([2, 0])
    run([0, 0])


@pytest.mark.functional
def test_meta_data_timestamp(dut: pytofcore.Sensor):
    def callback(measurement: pytofcore.Measurement, **kwargs):
        with callback.mutex:
            callback.count += 1
            #Note: for some reason it seems to take at least 3 measurement iterations 
            # for new settings to take effect this is probably a bug in the sensor
            # firmware but it's not import ATM
            if callback.count > 3:
                if measurement.data_type == measurement.DataType.DISTANCE and not callback.measurement:
                    callback.measurement = measurement

    callback.mutex = threading.Lock()
    callback.count = 0
    callback.measurement = None
    
    dut.subscribe_measurement(callback)
    dut.stream_distance()
    count = 0 # we will wait for upto 1 second in 0.1 second increments
    frames = 0
    last_timestamp = 0
    while frames < 3:
        count += 1
        time.sleep(0.1)
        with callback.mutex:
            if callback.measurement:
                assert (last_timestamp < callback.measurement.timestamp), "Received timestamp greater than last received timestamp."
                last_timestamp = callback.measurement.timestamp
                frames += 1
                callback.measurement = None # clear out the last measurement so we don't enter this block before another measurement is rx'd
        if count==10:
            break
    dut.stop_stream()


@pytest.mark.functional
def test_get_binning(dut: pytofcore.Sensor):
    binning_value_enabled = 3
    binning_value_disabled = 0

    dut.set_binning(1,1) # full binning
    binning_state = dut.get_binning()

    assert binning_state == binning_value_enabled

    dut.set_binning(0, 0) # no binning
    binning_state = dut.get_binning()

    assert binning_state == binning_value_disabled

    # new API for binning, takes only one boolean argument for full binning or no binning
    dut.set_binning(True)

    assert dut.get_binning() == binning_value_enabled

    dut.set_binning(False)

    assert dut.get_binning() == binning_value_disabled

@pytest.mark.functional
def test_rapid_commands(dut: pytofcore.Sensor):
    '''
    Verify that the protocol API and device can handle rapid command sequences while not streaming
    '''
    methods = [
        partial(dut.set_integration_time, 100),
        partial(dut.set_integration_time, 200),
        partial(dut.set_binning, True, True),
        partial(dut.set_binning, False, False),
        partial(dut.set_min_amplitude, 50), 
        partial(dut.set_min_amplitude, 15), 
        partial(dut.set_offset, 100),
        partial(dut.set_offset, 0),
        dut.get_sensor_info
        ]

    #Create a random sequence N calls to the functions listed above
    test_sequence = [random.choice(methods) for _ in range(2000)]

    try:
        for method in test_sequence: 
            method()
    except Exception as e:
        assert False, f'unexpected error occured: {e}'


@pytest.mark.functional
def test_rapid_commands_with_streaming(dut: pytofcore.Sensor):
    '''
    Verify that the protocol API and device can handle rapid command sequences while streaming
    '''
    
    def callback(m):
        callback.count += 1
    callback.count = 0

    dut.subscribe_measurement(callback)
    dut.stream_dcs_diff_ambient()

    methods = [
        lambda: setattr(dut, "modulation_frequency", 12000),
        partial(dut.set_integration_time, 100),
        partial(dut.set_min_amplitude, 50), 
        partial(dut.set_offset, 100),
        lambda: setattr(dut, "modulation_frequency", 24000),
        partial(dut.set_integration_time, 200),
        partial(dut.set_min_amplitude, 40), 
        partial(dut.set_offset, 50),
        ]

    #Create a random sequence N calls to the functions listed above
    test_sequence = [random.choice(methods) for _ in range(500)]
    
    try:
        for method in test_sequence: 
            method()
            time.sleep(0.001) # No UDP packets make it through unless commands take a breather

    except Exception as e:
         assert False, f'unexpected error occured: {e}'

    dut.stop_stream()
    assert callback.count != 0


@pytest.mark.functional
def test_sensor_location(dut: pytofcore.Sensor):

    sensor_location = dut.sensor_location

    dut.sensor_location = sensor_location

    time.sleep(1.0)

    assert sensor_location == dut.sensor_location


@pytest.mark.functional
def test_sensor_name(dut: pytofcore.Sensor):

    sensor_name = dut.sensor_name

    dut.sensor_name = sensor_name

    time.sleep(1.0)

    assert sensor_name == dut.sensor_name


@pytest.mark.functional
def test_get_vsm_number_of_elements(dut: pytofcore.Sensor):
    
    vsm_number_of_elements = dut.get_vsm_max_number_of_elements()
 
    assert vsm_number_of_elements is not None, "No VSM number of elements received"

    print("\n")
    print("VSM number of elements: " + str(vsm_number_of_elements))


@pytest.mark.functional
def test_vector_sequence_mode(dut: pytofcore.Sensor):

    mfkHz = 5501
    mfDeltakHz = 1545
    intTimeUs = 250
    intTimeDeltaUs = 100;
    
    new_elements = []
    max_size = 16
    for i in range(max_size):
        e = pytofcore.VsmElement()
        e.modulation_frequency = mfkHz
        mfkHz += mfDeltakHz
        e.integration_time = intTimeUs
        intTimeUs += intTimeDeltaUs
        new_elements.append(e)

    vsm = pytofcore.VsmControl()
    #vsm.m_numberOfElements = max_size
    vsm.elements = new_elements
    
    dut.set_vsm(vsm)

    vsm = dut.get_vsm()

    assert vsm
    assert len(vsm.elements) == max_size
    mfkHz = 5501
    intTimeUs = 250
    for i in range(max_size):
        e = vsm.elements[i]
        # Check returned modulation frequency value for rounding and range
        expectedKhz = (((mfkHz + 5) // 10)) * 10
        if expectedKhz > 24000:
            expectedKhz = 24000
        elif expectedKhz < 6000:
            expectedKhz = 6000
        assert e.modulation_frequency == expectedKhz
        mfkHz += mfDeltakHz
        # Check returned integration time value/range
        if intTimeUs > 4000:
            assert e.integration_time == 4000
        else:
            assert e.integration_time == intTimeUs
        intTimeUs += intTimeDeltaUs
        
    # Now turn VSM off
    vsm = pytofcore.VsmControl()
    dut.set_vsm(vsm)

    vsm = dut.get_vsm()

    assert vsm
    assert len(vsm.elements) == 0
 

@pytest.mark.functional
def test_stream_prior_to_set_vsm(dut: pytofcore.Sensor):
    #
    # This may seem like a strange test, but it exercises a scenario that was quite
    # a problem: sending a command (set_vsm) too soon after starting streaming
    #
    mfkHz = 5501
    mfDeltakHz = 1545
    intTimeUs = 250
    intTimeDeltaUs = 100;
    
    new_elements = []
    max_size = 16
    for i in range(max_size):
        e = pytofcore.VsmElement()
        e.modulation_frequency = mfkHz
        mfkHz += mfDeltakHz
        e.integration_time = intTimeUs
        intTimeUs += intTimeDeltaUs
        new_elements.append(e)

    vsm = pytofcore.VsmControl()
    #vsm.m_numberOfElements = max_size
    vsm.elements = new_elements
    
    def callback(m):
        callback.count += 1
    callback.count = 0

    dut.subscribe_measurement(callback)
    dut.stream_distance_amplitude()
    dut.set_vsm(vsm)

    vsm = dut.get_vsm()

    assert vsm
    assert len(vsm.elements) == max_size
        
    # Now turn VSM off
    vsm = pytofcore.VsmControl()
    dut.set_vsm(vsm)

    vsm = dut.get_vsm()

    assert vsm
    assert len(vsm.elements) == 0
   
    dut.stop_stream()
 

@pytest.mark.functional
def test_rapid_int_time_change_while_streaming(dut: pytofcore.Sensor):
     
    def callback(m):
        callback.count += 1
        
    callback.count = 0

    def set_int_time_loop():
        int_time = 1
        last_cmd_start_time = time.time()
        now = last_cmd_start_time
        end_time = now + 5.0
        time_btw_commands = 0.04
        while now < end_time:
            now = time.time()
            wait_time = (last_cmd_start_time + time_btw_commands) - now
            if wait_time > 0.0:
                time.sleep(wait_time)
                last_cmd_start_time = time.time()
                before = last_cmd_start_time
                
                dut.set_integration_time(int_time)
                
                after = time.time()
                cmd_duration = after - before
                if cmd_duration > 1.5:
                    print(f"Setting integration time {int_time}, Elapsed setting of int time {cmd_duration}")
                    raise RuntimeError("Setting integration time took > 1.5 seconds...")
                int_time += 1

    dut.subscribe_measurement(callback)
    
    dut.stream_dcs_diff_ambient()

    set_int_time_loop()
    
    dut.stop_stream()
 

@pytest.mark.functional
def test_rapid_imu_query_while_streaming(dut: pytofcore.Sensor):
     
    def callback(m):
        callback.count += 1
        
    callback.count = 0

    def query_imu():
        last_cmd_start_time = time.time()
        now = last_cmd_start_time
        end_time = now + 5.0
        time_btw_commands = 0.04
        while now < end_time:
            now = time.time()
            wait_time = (last_cmd_start_time + time_btw_commands) - now
            if wait_time > 0.0:
                time.sleep(wait_time)
                last_cmd_start_time = time.time()
                before = last_cmd_start_time
                
                res = dut.get_imu_data()
                
                after = time.time()
                cmd_duration = after - before
                if cmd_duration > 1.5:
                    print(f"Getting IMU data took {cmd_duration}")
                    raise RuntimeError("Setting integration time took > 1.5 seconds...")

    dut.subscribe_measurement(callback)
    
    dut.stream_dcs_diff_ambient()

    query_imu()
    
    dut.stop_stream()


@pytest.mark.functional
def test_frame_crc_state(dut: pytofcore.Sensor):
    
    original_crc_state = dut.get_frame_crc_state()
    
    dut.set_frame_crc_state(0)
    crc_state = dut.get_frame_crc_state()
    assert crc_state is not None, "No CRC state"
    assert crc_state == (0), "Failed to disable CRCs"
    
    dut.set_frame_crc_state(1)
    crc_state = dut.get_frame_crc_state()
    assert crc_state is not None, "No CRC state"
    assert crc_state == (1), "Failed to enable CRCs"
    
    dut.set_frame_crc_state(2)
    crc_state = dut.get_frame_crc_state()
    assert crc_state is not None, "No CRC state"
    assert crc_state == (2), "Failed to enable per-frame CRCs"
     
    dut.set_frame_crc_state(original_crc_state)
    crc_state = dut.get_frame_crc_state()
    assert crc_state is not None, "No CRC state"
    assert crc_state == (original_crc_state), "Failed to restore CRC state"
   
