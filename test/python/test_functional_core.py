import time
import struct
import pytest
import pytofcore
import threading
import random
import ipaddress
from typing import List
from functools import partial

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
def test_stream_distance_amplitude_frames(dut: pytofcore.Sensor):
    def callback(measurement: pytofcore.Measurement, **kwargs):
        if not callback.measurement and (measurement.data_type == pytofcore.Measurement.DataType.DISTANCE_AMPLITUDE):
            callback.measurement = measurement

    callback.measurement = None
    dut.subscribe_measurement(callback)
    dut.stream_distance_amplitude()
    
    time.sleep(1.0)
    assert callback.measurement is not None, "No Distance & Amplitude measurements received"


@pytest.mark.functional
def test_stream_distance(dut: pytofcore.Sensor):

    def callback(measurement: pytofcore.Measurement, **kwargs):
        if not callback.measurement and (measurement.data_type == pytofcore.Measurement.DataType.DISTANCE):
            callback.measurement = measurement

    callback.measurement = None
    dut.subscribe_measurement(callback)
    dut.stream_distance()
    
    time.sleep(1.0)
    assert callback.measurement is not None, "No Distance measurements received"


@pytest.mark.functional
def test_stream_dcs(dut: pytofcore.Sensor):

    def callback(measurement: pytofcore.Measurement, **kwargs):
        if not callback.measurement and (measurement.data_type == pytofcore.Measurement.DataType.DCS):
            callback.measurement = measurement

    callback.measurement = None
    dut.subscribe_measurement(callback)
    dut.stream_dcs()
    
    time.sleep(1.0)
    assert callback.measurement is not None, "No DCS measurements received"


@pytest.mark.functional
def test_stream_dcs_ambient(dut: pytofcore.Sensor):
    
    def callback(measurement: pytofcore.Measurement, **kwargs):
        if not callback.dcs_measurement and (measurement.data_type == pytofcore.Measurement.DataType.DCS):
            callback.dcs_measurement = measurement
        if not callback.ambient_measurement and (measurement.data_type == pytofcore.Measurement.DataType.AMBIENT):
            callback.ambient_measurement = measurement

    callback.dcs_measurement = None
    callback.ambient_measurement = None
    dut.subscribe_measurement(callback)
    dut.stream_dcs_ambient()
    time.sleep(1.0)
    assert callback.dcs_measurement is not None, "No DCS measurements received"
    assert callback.ambient_measurement is not None, "No ambient measurements received"


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

    assert versionInfo._fields == ('deviceSerialNumber', 'cpuBoardSerialNumber', 'illuminatorBoardSerialNumber', 'modelName', 'lastResetType', 'softwareId', 'softwareVersion', 'cpuVersion', 'chipId', 'illuminatorSwVersion', 'illuminatorSwId', 'illuminatorHwCfg' ,'backpackModule')


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

    assert settings == dut.ipv4_settings


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
def test_meta_data_integration_times(dut: pytofcore.Sensor):

    def run(TEST_VALUES: List[int]):
        def callback(measurement: pytofcore.Measurement, **kwargs):
            with callback.mutex:
                callback.count += 1
                #Note: for some reason it seems to take at least 2 measurement iterations 
                # for new integration time settings to take effect
                # this is probably a bug in the sensor firmware but it's not import ATM
                if callback.count > 2:
                    if measurement.data_type == measurement.DataType.DISTANCE and not callback.measurement:
                        callback.measurement = measurement

        callback.mutex = threading.Lock()
        callback.count = 0
        callback.measurement = None
        dut.set_integration_times(*TEST_VALUES)
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
        int_times = callback.measurement.integration_times
        assert int_times, "No integration time data included with the measurement"
        assert len(int_times) == 3, "Not enough integration time values in the meta-data"
        assert TEST_VALUES == int_times, "Incorrect integration time values included in meta-data"
        int_times = dut.get_integration_times()
        assert TEST_VALUES == int_times, "Incorrect integration time values from get_integration_times()"

    run([11, 22, 33])
    run([100, 0, 0])
    run([111, 222, 333])


@pytest.mark.functional
def test_meta_data_modulation_frequencies(dut: pytofcore.Sensor):

    def run(freq_khz_to_set: int, freq_khz_expected):
        def callback(measurement: pytofcore.Measurement, **kwargs):
            with callback.mutex:
                callback.count += 1
                #Note: for some reason it seems to take at least 2 measurement iterations 
                # for new integration time settings to take effect
                # this is probably a bug in the sensor firmware but it's not import ATM
                if callback.count > 1:
                    if measurement.data_type == measurement.DataType.DISTANCE and not callback.measurement:
                        callback.measurement = measurement

        callback.mutex = threading.Lock()
        callback.count = 0
        callback.measurement = None

        dut.modulation_frequency = freq_khz_to_set
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

        mod_freqs = callback.measurement.modulation_frequencies
        assert mod_freqs, "No modulation frequency data included with the measurement"
        assert len(mod_freqs) == 1, "Not enough modulation frequency values in the meta-data"
        assert freq_khz_expected == mod_freqs[0]/1000, "Incorrect modulation frequency values included in meta-data"

    run(700, 6000)
    run(7500, 6000)
    run(15000, 12000)
    run(30000, 24000)
    run(6000, 6000)
    run(12000, 12000)
    run(24000, 24000)


@pytest.mark.functional
def test_meta_data_binning(dut: pytofcore.Sensor):

    def run(TEST_VALUE: List[int]):
        def callback(measurement: pytofcore.Measurement, **kwargs):
            with callback.mutex:
                callback.count += 1
                #Note: for some reason it seems to take at least 2 measurement iterations 
                # for new settings to take effect this is probably a bug in the sensor
                # firmware but it's not import ATM
                if callback.count > 2:
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
        count = 0 # we will wait for upto 1 second in 0.1 second increments
        while count != 10:
            count += 1
            time.sleep(0.1)
            with callback.mutex:
                if callback.measurement:
                    count = 10
        dut.stop_stream()
        
        assert callback.measurement, "No measurement received"

        horizontal_binning = callback.measurement.horizontal_binning
        assert horizontal_binning is not None, "No horizontal binning data included with the measurement"
        assert TEST_VALUE[0] == horizontal_binning, "Incorrect horizontal binning value included in meta-data"

        vertical_binning = callback.measurement.vertical_binning
        assert vertical_binning is not None, "No vertical binning data included with the measurement"
        assert TEST_VALUE[1] == vertical_binning, "Incorrect vertical binning value included in meta-data"

    run([2, 2])
    run([0, 2])
    run([2, 0])
    run([0, 0])


@pytest.mark.functional
def test_rapid_commands(dut: pytofcore.Sensor):
    '''
    Verify that the protocol API and device can handle rapid command sequences while not streaming
    '''
    methods = [
        partial(dut.set_integration_times, 100, 0, 0),
        partial(dut.set_integration_times, 200, 0, 0),
        partial(dut.set_binning, True, True),
        partial(dut.set_binning, False, False),
        partial(dut.set_min_amplitude, 50), 
        partial(dut.set_min_amplitude, 5), 
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
    dut.stream_distance_amplitude()

    methods = [
        lambda: setattr(dut, "modulation_frequency", 12000),
        partial(dut.set_integration_times, 100, 0, 0),
        partial(dut.set_min_amplitude, 50), 
        partial(dut.set_offset, 100),
        lambda: setattr(dut, "modulation_frequency", 24000),

        ]

    #Create a random sequence N calls to the functions listed above
    test_sequence = [random.choice(methods) for _ in range(500)]
    
    try:
        for method in test_sequence: 
            method()
    except Exception as e:
         assert False, f'unexpected error occured: {e}'

    dut.stop_stream()
    assert callback.count != 0


@pytest.mark.functional
def test_sensor_location(dut: pytofcore.Sensor):

    sensor_location = dut.sensor_location

    dut.sensor_location = sensor_location

    assert sensor_location == dut.sensor_location


@pytest.mark.functional
def test_sensor_name(dut: pytofcore.Sensor):

    sensor_name = dut.sensor_name

    dut.sensor_name = sensor_name

    assert sensor_name == dut.sensor_name

