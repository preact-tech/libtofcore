import pytest
import pytofcore


#Mark this test as expected to fail because we really
# don't have a reliable method for setting the module version
@pytest.mark.xfail
def test_module():
    assert pytofcore.__version__ == "0.0.1"


def test_sensor_class():
    """This Test is meant to run without a device connected to PC"""
    with pytest.raises(Exception):
        pytofcore.Sensor()
