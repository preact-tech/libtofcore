def pytest_addoption(parser):
    try:
        parser.addoption(
            '--sensor-port-name', action='store', default=None, help='Device name for connecting to the sensor under test'
        )
        parser.addoption(
            '--sensor-baud-rate', action='store', default=None, help='Baud rate to use when connecting to the sensor under test'
        )
    except ValueError:
        #Ignore ValueErrors due to adding the option more than once in different config files.
        pass

def pytest_configure(config):
    config.addinivalue_line(
        "markers", "functional: Tests to run with functional sensor connected"
    )