def pytest_addoption(parser):
    try:
        parser.addoption(
            '--sensor-uri', action='store', default=None, help='URI to use when connecting to the device'
        )
    except ValueError:
        #Ignore ValueErrors due to adding the option more than once in different config files.
        pass

def pytest_configure(config):
    config.addinivalue_line(
        "markers", "functional: Tests to run with functional sensor connected"
    )