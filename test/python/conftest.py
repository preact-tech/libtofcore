def pytest_addoption(parser):
    parser.addoption(
        '--sensor-port-name', action='store', default=None, help='Device name for connecting to the sensor under test'
    )
    parser.addoption(
        '--sensor-baud-rate', action='store', default=None, help='Baud rate to use when connecting to the sensor under test'
    )