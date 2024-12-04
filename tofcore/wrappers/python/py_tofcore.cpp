#include "tof_sensor.hpp"
#include "device_discovery.hpp"
#include <pybind11/functional.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)


constexpr auto DCS_DATA_DOCSTRING =
  "Obtain all 4 dcs frames in the measurement in a single MemoryView\n"
  "\n"
  "Data is packed in frame then row major order with Numpy shape (4, 240, 320)\n"
  "DCS pixels are encoded as 16 bit signed little endian values with 12-bits\n"
  "of precision. Bit 12 is used to encode a saturation flag such that a saturated\n"
  "pixel's bit-12 will be the opposite of the sign bit.";

constexpr auto DCS_DIFF_DATA_DOCSTRING =
  "Obtain both (2) DCS difference frames in the measurement in a single MemoryView\n"
  "\n"
  "Data is packed in frame then row major order with Numpy shape (2, 240, 320)\n"
  "DCS differences are encoded as 16 bit signed little endian values with 12-bits\n"
  "of precision. If only BIT 12 is set (0x1000), then no data is available because\n"
  "at least on pixel is saturated. If only BIT 13 is set (0x2000) then no data is available\n"
  "because at least one of the pixels read as 0. If only BIT 14 is set (0x4000) then no\n"
  "is available because at least one of the pixels read as 0xFFF.";

constexpr auto ILLUMINATOR_INFO_DOCSTRING =
  "Obtain infromation on the illuminators state for the measurement\n"
  "\n"
  "The following information is available:\n"
  " - led_segments_enabled : A bit mask indicating which segments are enabled, one bit per segment\n"
  "   (note some types of sensors have only 1 or 2 segments and each segment can have multiple LEDs)\n"
  " - temperature_c : Temperature of the illuminator board and LEDs.\n"
  " - vled_v : Voltage applied to the LEDs\n"
  " - photodiode_v : Voltage read from the photodiode sensor near the LEDs.";

namespace py = pybind11;

#define PyCollectionsModule (ImportCollections())
#define PyNamedTuple (ImportNamedTuple())
#define PyIpaddressModule (ImportIpaddress())
#define PyIPv4Interface (ImportIPv4Interface())
#define PyIPv4Address (ImportIPv4Address())
#define PyIPv4Settings (IPv4SettingsType())
#define PyIPv4Endpoint (IPv4EndpointType())
#define PyIPv4LogSettings (IPv4LogSettingsType())

inline const py::module_& ImportCollections() {
    static const py::module_* ptr = new py::module_{py::module_::import("collections")};
    return *ptr;
}
inline const py::object& ImportNamedTuple() {
    static const py::object* ptr = new py::object{py::getattr(PyCollectionsModule, "namedtuple")};
    return *ptr;
}
inline const py::module_& ImportIpaddress() {
    static const py::module_* ptr = new py::module_{py::module_::import("ipaddress")};
    return *ptr;
}
inline const py::object& ImportIPv4Interface() {
    static const py::object* ptr = new py::object{py::getattr(PyIpaddressModule, "IPv4Interface")};
    return *ptr;
}
inline const py::object& ImportIPv4Address() {
    static const py::object* ptr = new py::object{py::getattr(PyIpaddressModule, "IPv4Address")};
    return *ptr;
}

inline const py::object& IPv4SettingsType() {
    static const auto type = []() {
        auto namedTuple_attr = PyNamedTuple;
        py::list fields;
        fields.append("interface");
        fields.append("gateway");
        return namedTuple_attr("IPv4Settings", fields);
    }();
    return type;
}

inline const py::object& IPv4EndpointType() {
    static const auto type = []() {
        auto namedTuple_attr = PyNamedTuple;
        py::list fields;
        fields.append("address");
        fields.append("port");
        return namedTuple_attr("IPv4Endpoint", fields);
    }();
    return type;
}

inline const py::object& IPv4LogSettingsType() {
    static const auto type = []() {
        auto namedTuple_attr = PyNamedTuple;
        py::list fields;
        fields.append("address");
        fields.append("port");
        return namedTuple_attr("IPv4LogSettings", fields);
    }();
    return type;
}

/// @brief Wrap python callback with lambda that will catch exceptions and handle them safely
/// @param py_callback client callback functor
static void subscribeMeasurement(tofcore::Sensor& s, tofcore::Sensor::on_measurement_ready_t py_callback)
{
    auto f = [py_callback](auto measurement)
    {
        py::gil_scoped_acquire aquire;
        py::scoped_estream_redirect redirect;
        try {
            py_callback(measurement);
        } catch(py::error_already_set& e) {
            std::cerr << "uncaught exception from user callback:\n" << e.what() << std::flush;
            #if PYBIND11_VERSION_MAJOR >= 2 && PYBIND11_VERSION_MINOR >= 6
            e.discard_as_unraisable("uncaught exception from user callback");
            #endif
        }
    };
    s.subscribeMeasurement(f);
}

static auto getImuData(tofcore::Sensor& s)
{
    //Use static and a lambda to create the ImuData_type namedtuple type only once.
    static auto ImuData_type = []() {
        auto namedTuple_attr = pybind11::module::import("collections").attr("namedtuple");
        py::list fields;
        fields.append("accel_millig");
        fields.append("gyro_milliDegreesPerSecond");
        fields.append("temperature_milliDegreesC");
        fields.append("timestamp");
        return namedTuple_attr("ImuData", fields);
    }();

    std::optional<TofComm::ImuScaledData_T> result;
    {
        py::gil_scoped_release gsr;
        result = s.getImuInfo();
    }
    if (!result)
    {
        throw std::runtime_error("An error occured while getting Imu information");
    }

    return ImuData_type(result->accelerometer_millig, 
                        result->gyro_milliDegreesPerSecond, 
                        result->temperature_milliDegreesC,
                        result->timestamp);
}


static auto getIPv4Settings(tofcore::Sensor& s)
{
    std::array<std::byte, 4> ipv4Address;
    std::array<std::byte, 4> ipv4Mask;
    std::array<std::byte, 4> ipv4Gateway;

    bool result = false;
    {
        py::gil_scoped_release gsr;
        result = s.getIPv4Settings(ipv4Address, ipv4Mask, ipv4Gateway);
    }
    if(!result)
    {
        throw std::runtime_error("An error ocurred while getting IPv4 settings");
    }

    std::stringstream if_ss;
    if_ss <<
        (int)ipv4Address[0] << '.' << (int)ipv4Address[1] << '.' << (int)ipv4Address[2] << '.' << (int)ipv4Address[3] << '/' <<
        (int)ipv4Mask[0] << '.' << (int)ipv4Mask[1] << '.' << (int)ipv4Mask[2] << '.' << (int)ipv4Mask[3];

    std::stringstream gw_ss;
    gw_ss << (int)ipv4Gateway[0] << '.' << (int)ipv4Gateway[1] << '.' << (int)ipv4Gateway[2] << '.' << (int)ipv4Gateway[3];

    return PyIPv4Settings(
        PyIPv4Interface(py::str(if_ss.str())),
        PyIPv4Address(py::str(gw_ss.str())));
}

static void setIPv4Settings(tofcore::Sensor &s, py::object& settings)
{
    if(!py::isinstance(settings, PyIPv4Settings))
    {
        throw pybind11::type_error("IPv4Settings must be of type pytofcore.IPv4Settings");
    }
    auto interface = settings.attr("interface");
    auto gateway = settings.attr("gateway");
    if(!py::isinstance(interface, PyIPv4Interface))
    {
        throw pybind11::type_error("IPv4Settings.interface attribute must be of type ipaddress.IPv4Interface");
    }
    if(!py::isinstance(gateway, PyIPv4Address))
    {
        throw pybind11::type_error("IPv4Settings.gateway attribute must be of type ipaddress.IPv4Address");
    }

    auto py_ip_to_array = [](const py::object& ip) -> std::array<std::byte, 4>
    {
        py::bytes bytes = ip.attr("packed");
        auto sv = std::string_view(bytes);
        return std::array<std::byte, 4>{std::byte(sv[0]), std::byte(sv[1]), std::byte(sv[2]), std::byte(sv[3])};
    };

    auto ipv4Address = py_ip_to_array(interface.attr("ip"));
    auto ipv4Mask = py_ip_to_array(interface.attr("network").attr("netmask"));
    auto ipv4Gateway = py_ip_to_array(settings.attr("gateway"));

    bool ok = false;
    {
        py::gil_scoped_release gsr;
        ok = s.setIPv4Settings(ipv4Address, ipv4Mask, ipv4Gateway);
    }
    if(!ok)
    {
        throw std::runtime_error("An error occurred setting IPv4 settings");
    }
}


static auto getIPMeasurementEndpoint(tofcore::Sensor &s)
{

    auto result = [&]() {
        py::gil_scoped_release gsr;
        return s.getIPMeasurementEndpoint();
    } ();

    if(!result)
    {
        throw std::runtime_error("An error ocurred while getting IP measurement endpoint");
    }

    auto ipv4Address = std::get<0>(*result);
    auto port = std::get<1>(*result);

    std::stringstream ss;
    ss << (int)ipv4Address[0] << '.' << (int)ipv4Address[1] << '.' << (int)ipv4Address[2] << '.' << (int)ipv4Address[3];

    return PyIPv4Endpoint(
        PyIPv4Address(py::str(ss.str())),
        port);
}


static void setIPMeasurementEndpoint(tofcore::Sensor &s, py::object& endpoint)
{
    if(!py::isinstance(endpoint, PyIPv4Endpoint))
    {
        throw pybind11::type_error("IPv4MeasurementEndpoint must be of type pytofcore.IPv4Endpoint");
    }
    auto address = PyIPv4Address(endpoint.attr("address"));
    auto port = py::int_(endpoint.attr("port"));

    auto py_ip_to_array = [](const py::object& ip) -> std::array<std::byte, 4>
    {
        py::bytes bytes = ip.attr("packed");
        auto sv = std::string_view(bytes);
        return std::array<std::byte, 4>{std::byte(sv[0]), std::byte(sv[1]), std::byte(sv[2]), std::byte(sv[3])};
    };

    auto a = py_ip_to_array(address);
    auto ok = false;
    {
        py::gil_scoped_release gsr;
        ok = s.setIPMeasurementEndpoint(a, port);
    }
    if(!ok)
    {
        throw std::runtime_error("An error occurred setting IP measurement endpoint");
    }
}


static auto getIPv4LogSettings(tofcore::Sensor& s)
{
    std::optional<std::tuple<std::array<std::byte, 4>, uint16_t>> result;
    {
        py::gil_scoped_release gsr;
        result = s.getLogIpv4Destination();
    }
    if(!result)
    {
        throw std::runtime_error("An error occurred while getting Logging IPv4 settings");
    }

    auto ipv4Address = std::get<0>(*result);
    uint16_t udpPort = std::get<1>(*result);

    std::stringstream adrs_ss;
    adrs_ss <<
        (int)ipv4Address[0] << '.' << (int)ipv4Address[1] << '.' << (int)ipv4Address[2] << '.' << (int)ipv4Address[3];

    return PyIPv4LogSettings(PyIPv4Address(py::str(adrs_ss.str())), udpPort);
}


static void setIPv4LogSettings(tofcore::Sensor &s, py::object& settings)
{
    if(!py::isinstance(settings, PyIPv4LogSettings))
    {
        throw pybind11::type_error("IPv4LogSettings must be of type pytofcore.IPv4LogSettings");
    }
    auto address = settings.attr("address");
    auto port = py::int_(settings.attr("port"));
    if(!py::isinstance(address, PyIPv4Address))
    {
        throw pybind11::type_error("IPv4LogSettings.address attribute must be of type ipaddress.IPv4Interface");
    }

    auto py_ip_to_array = [](const py::object& ip) -> std::array<std::byte, 4>
    {
        py::bytes bytes = ip.attr("packed");
        auto sv = std::string_view(bytes);
        return std::array<std::byte, 4>{std::byte(sv[0]), std::byte(sv[1]), std::byte(sv[2]), std::byte(sv[3])};
    };

    auto ipv4Address = py_ip_to_array(settings.attr("address"));

    bool ok = false;
    {
        py::gil_scoped_release gsr;
        ok = s.setLogIPv4Destination(ipv4Address, port);
    }
    if(!ok)
    {
        throw std::runtime_error("An error occurred setting Log IPv4 settings");
    }
}


static auto getLensInfo(tofcore::Sensor& s)
{
    //Use static and a lambda to create the LensInfo_type namedtuple type only once.
    static auto LensInfo_type = []() {
        auto namedTuple_attr = pybind11::module::import("collections").attr("namedtuple");
        py::list fields;
        fields.append("rowOffset");
        fields.append("columnOffset");
        fields.append("rowFocalLength");
        fields.append("columnFocalLength");
        fields.append("undistortionCoeffs");
        fields.append("hfov");
        fields.append("vfov");
        return namedTuple_attr("LensInfo", fields);
    }();
    std::optional<tofcore::LensIntrinsics_t> result;
    {
        py::gil_scoped_release gsr;
        result = s.getLensIntrinsics();
    }
    if (!result)
    {
        throw std::runtime_error("An error occured while getting Lens information");
    }

    return LensInfo_type(result->m_rowOffset, result->m_columnOffset, result->m_rowFocalLength,
                         result->m_columnFocalLength, result->m_undistortionCoeffs, result->m_hfov, result->m_vfov);
}

static auto getPixelRays(tofcore::Sensor& s)
{
    //Use static and a lambda to create the PixelRays namedtuple type only once.
    static auto PixelRays_type = []() {
        auto namedTuple_attr = pybind11::module::import("collections").attr("namedtuple");
        py::list fields;
        fields.append("x");
        fields.append("y");
        fields.append("z");
        return namedTuple_attr("PixelRays", fields);
    }();

    std::vector<double> rays_x;
    std::vector<double> rays_y;
    std::vector<double> rays_z;

    bool result {true};
    {
        py::gil_scoped_release gsr;
        result = s.getLensInfo(rays_x, rays_y, rays_z);
    }
    if(!result)
    {
        throw std::runtime_error("An error occured while getting pixel ray information");
    }

    return PixelRays_type(rays_x, rays_y, rays_z);
}

static auto getSensorInfo(tofcore::Sensor& s)
{

    //Use static and a lambda to create the VersionData namedtuple type only once.
    static auto VersionData_type = []() {
        auto namedTuple_attr = pybind11::module::import("collections").attr("namedtuple");
        py::list fields;
        fields.append("deviceSerialNumber");
        fields.append("cpuBoardSerialNumber");
        fields.append("illuminatorBoardSerialNumber");
        fields.append("modelName");
        fields.append("lastResetType");

        fields.append("softwareId");
        fields.append("softwareVersion");

        // Mojave platform only
        fields.append("cpuVersion");
        fields.append("chipId");
        fields.append("illuminatorSwVersion");
        fields.append("illuminatorSwId");
        fields.append("illuminatorHwCfg");
        fields.append("backpackModule");

        return namedTuple_attr("VersionData", fields);
    }();

    TofComm::versionData_t versionData;

    bool result = false;
    {
        py::gil_scoped_release gsr;
        result = s.getSensorInfo(versionData);
    }

    if (!result)
    {
        throw std::runtime_error("An error occurred trying to read sensor version info");
    }

    // Extract packed value
    uint32_t chipId { versionData.m_sensorChipId };

    return VersionData_type(versionData.m_deviceSerialNumber,
                            versionData.m_cpuBoardSerialNumber,
                            versionData.m_illuminatorBoardSerialNumber,
                            versionData.m_modelName,
                            versionData.m_lastResetType,
                            versionData.m_softwareSourceID,
                            versionData.m_softwareVersion,
                            versionData.m_cpuBoardRevision,
                            chipId,
                            versionData.m_illuminatorSwVersion,
                            versionData.m_illuminatorSwSourceId,
                            versionData.m_illuminatorHwCfg,
                            (uint8_t)versionData.m_backpackModule);
}

static auto getSensorLocation(tofcore::Sensor& s)
{
    py::gil_scoped_release gsr;

    auto sensorLocation = s.getSensorLocation();
    if(!sensorLocation)
    {
        throw std::runtime_error("An error occured while getting the sensor location");
    }
    return *sensorLocation;
}

static void setSensorLocation(tofcore::Sensor &s, std::string& location)
{
    py::gil_scoped_release gsr;

    const auto ok = s.setSensorLocation(location);
    if(!ok)
    {
        throw std::runtime_error("An error occurred setting sensor location");
    }
}

static auto getSensorName(tofcore::Sensor& s)
{
    py::gil_scoped_release gsr;

    auto sensorName = s.getSensorName();
    if(!sensorName)
    {
        throw std::runtime_error("An error occured while getting the sensor name");
    }
    return *sensorName;
}

static void setSensorName(tofcore::Sensor &s, std::string& name)
{
    py::gil_scoped_release gsr;

    const auto ok = s.setSensorName(name);
    if(!ok)
    {
        throw std::runtime_error("An error occurred setting sensor name");
    }
}

static auto getSensorControlStatus(tofcore::Sensor& s)
{
    auto tof_control_status = s.getSensorControlState();
    if (!tof_control_status)
    {
        throw std::runtime_error("An error occured while getting tof controller status ");
    }

    return  tof_control_status.value();
}

static auto getSensorStatus(tofcore::Sensor& s)
{
    //Use static and a lambda to create the sensorStatus namedtuple type only once.
    static auto Sensor_Status_t = []() {
        auto namedTuple_attr = pybind11::module::import("collections").attr("namedtuple");
        py::list fields;
        fields.append("lastTemperature");
        fields.append("USB_Current");
        fields.append("BIT_Status");

        return namedTuple_attr("SensorStatus", fields);
    }();

    TofComm::Sensor_Status_t sensorStatus;

    bool result = false;
    {
        py::gil_scoped_release gsr;
        result = s.getSensorStatus(sensorStatus);
    }
    if (!result)
    {
        throw std::runtime_error("An error occurred trying to read sensor status info");
    }

    // Extract packed value
    int16_t  lastTemperature { sensorStatus.lastTemperature };
    int16_t  USB_Current     { sensorStatus.USB_Current};
    uint32_t BIT_Status      { sensorStatus.BIT_Status};

    // return natural units, degree Celsius for temperature and amperes for current.
    return Sensor_Status_t(lastTemperature/100.0,
                           USB_Current/1000.0,
                           BIT_Status);

}

/// @brief  Helper function to read the integration times from a sensor such that an
///         exception is thrown if the read fails.
static auto getSensorIntegrationTime(tofcore::Sensor& s)
{
    auto integrationTime = s.getIntegrationTime();
    if (!integrationTime)
    {
        throw std::runtime_error("An error occured while getting integration time");
    }

    return integrationTime;
}

/// @brief  Helper function to read the integration time limits from a sensor such that an
///         exception is thrown if the read fails.
static auto getSensorIntegrationTimeAndLimits(tofcore::Sensor& s)
{
    auto integrationTimeLimits = s.getIntegrationTimeUsAndLimits();
    if (!integrationTimeLimits)
    {
        throw std::runtime_error("An error occured while getting integration time limits");
    }

    return integrationTimeLimits;
}

/// @brief  Helper function to read the modulation frequency limits and step size from a sensor
///         such that an exception is thrown if the read fails.
static auto getModulationFrequencyAndLimitsAndStepSize(tofcore::Sensor& s)
{
    auto modulationFreqLimitsAndStepSize = s.getModulationFreqKhzAndLimitsAndStepSize();
    if (!modulationFreqLimitsAndStepSize)
    {
        throw std::runtime_error("An error occured while getting integration time limits");
    }

    return modulationFreqLimitsAndStepSize;
}

/// @brief  Helper function to read the frame period limits from a sensor such that an
///         exception is thrown if the read fails.
static auto getSensorFramePeriodAndLimits(tofcore::Sensor& s)
{
    auto framePeriodLimits = s.getFramePeriodMsAndLimits();
    if (!framePeriodLimits)
    {
        throw std::runtime_error("An error occured while getting frame period limits");
    }

    return framePeriodLimits;
}

/// @brief  Helper function to read the min amplitude limits from a sensor such that an
///         exception is thrown if the read fails.
static auto getSensorMinAmplitudeAndLimits(tofcore::Sensor& s)
{
    auto minAmplitudeLimits = s.getMinAmplitudeAndLimits();
    if (!minAmplitudeLimits)
    {
        throw std::runtime_error("An error occured while getting min amplitude limits");
    }

    return minAmplitudeLimits;
}

/// @brief  Helper function to read the vled setting and max/min vled limits from a sensor such that an
///         exception is thrown if the read fails.
static auto getSensorVledSettingAndLimits(tofcore::Sensor& s)
{
    auto vledSettingLimits = s.getVledSettingAndLimits();
    if (!vledSettingLimits)
    {
        throw std::runtime_error("An error occured while getting vled setting and limits");
    }

    return vledSettingLimits;
}

/// @brief Helper function to obtain a memoryview of the distance data in
///        a Measurement object
static auto get_distance_view(const tofcore::Measurement_T &m)
{
    const auto view = m.distance();
    const auto* ptr = view.data();
    const auto element_size = sizeof(*ptr);
    return py::memoryview::from_buffer(
        ptr,
        {m.height(), m.width()},
        {element_size * m.width(), element_size}
    );
}

/// @brief Helper function to obtain a memoryview of the distance data in
///        a Measurement object
static auto get_amplitude_view(const tofcore::Measurement_T &m)
{
    const auto view = m.amplitude();
    const auto* ptr = view.data();
    const auto element_size = sizeof(*ptr);
    return py::memoryview::from_buffer(
        ptr,
        {m.height(), m.width()},
        {element_size * m.width(), element_size}
    );
}


/// @brief Helper function to obtain a memoryview of the ambient data in
///   a Measurement object
static auto get_ambient_view(const tofcore::Measurement_T &m)
{
    const auto view = m.ambient();
    const auto* ptr = view.data();
    const auto element_size = sizeof(*ptr);
    return py::memoryview::from_buffer(
        view.data(),
        {m.height(), m.width()},
        {element_size * m.width(), element_size}
    );
}


/// @brief Helper function to obtain a memoryview of the DCS data in
///        a Measurement object
static auto get_dcs_view(const tofcore::Measurement_T &m)
{
    const auto view = m.pixel_buffer();
    const auto ptr = (int16_t*)view.data();
    const auto element_size = sizeof(*ptr);
    return py::memoryview::from_buffer(
        ptr,
        {4, m.height(), m.width()},
        {element_size * m.width() * m.height(), element_size * m.width(), element_size}
    );
}


/// @brief Helper function to obtain a memoryview of the DCS difference data in
///        a Measurement object
static auto get_dcs_diff_view(const tofcore::Measurement_T &m)
{
    const auto view = m.pixel_buffer();
    const auto ptr = (int16_t*)view.data();
    const auto element_size = sizeof(*ptr);
    return py::memoryview::from_buffer(
        ptr,
        {2, m.height(), m.width()},
        {element_size * m.width() * m.height(), element_size * m.width(), element_size}
    );
}


/// @brief Helper function to obtain a memoryview of the raw meta-data associated with
///   a Measurement instance.
static auto get_meta_data_view(const tofcore::Measurement_T &m)
{
    const auto view = m.pixel_buffer();
    return py::memoryview::from_memory(view.data(), view.size());
}


/// @brief Helper function to obtain a DLL settings from the measurement header
static py::object get_dll_settings(const tofcore::Measurement_T &m)
{
    //Use static and a lambda to create the ChipInfo namedtuple type only once.
    static auto DllSettings_type = []() {
        auto namedTuple_attr = pybind11::module::import("collections").attr("namedtuple");
        py::list fields;
        fields.append("enabled");
        fields.append("coarse_step");
        fields.append("fine_step");
        fields.append("finest_step");
        return namedTuple_attr("DllSettings", fields);
    }();

    const auto settings = m.dll_settings();
    if(!settings)
    {
        return py::cast<py::none>(Py_None);
    }
    return DllSettings_type((*settings)[0] != 0, (*settings)[1], (*settings)[2], (*settings)[3] );
}

/// @brief Helper function
static py::object get_illuminator_info(const tofcore::Measurement_T &m)
{
    //Use static and a lambda to create namedtuple type only once.
    static auto Tuple_type = []() {
        auto namedTuple_attr = pybind11::module::import("collections").attr("namedtuple");
        py::list fields;
        fields.append("led_segments_enabled");
        fields.append("temperature_c");
        fields.append("vled_v");
        fields.append("photodiode_v");
        return namedTuple_attr("IlluminatorInfo", fields);
    }();

    const auto info = m.illuminator_info();
    if(!info)
    {
        return py::cast<py::none>(Py_None);
    }
    return Tuple_type((*info).led_segments_enabled,
                      (*info).temperature_c,
                      (*info).vled_v,
                      (*info).photodiode_v);
}

static std::optional<uint16_t> modulation_get(tofcore::Sensor &sensor)
{
    py::gil_scoped_release gsr;

    auto modulation = sensor.getModulation();
    if (modulation)
    {
        return modulation;
    }
    else
    {
        throw std::runtime_error("An error occurred attempting to read modulation frequency state.");
    }
}

static bool modulation_set(tofcore::Sensor &sensor, uint16_t modFreqkHz)
{
    py::gil_scoped_release gsr;
    return sensor.setModulation(modFreqkHz);
}


/// @brief Deprecated version of modulation frequency set.
/// This is just here to support the old method of setting modulation frequency using enumerated value
/// where index values represented specific frequencies: 0 == 12mHz, 1 == 24mHz, 2 = 6mHz
/// @return boolean indicating success or false
static bool modulation_set_deprecated(tofcore::Sensor &sensor, int index, int channel)
{
    (void)channel; //No longer used.

    uint16_t modFreqkHz = 0;
    switch (index)
    {
    case 0: //12mHz
        modFreqkHz = 12000;
        break;
    case 1: //24mHz
        modFreqkHz = 24000;
        break;
    case 2: //24mHz
        modFreqkHz = 6000;
        break;
    default:
        return false;
        break;
    }
    return modulation_set(sensor, modFreqkHz);
}


static bool hflip_get(tofcore::Sensor &sensor)
{
    py::gil_scoped_release gsr;
    auto hflip = sensor.isFlipHorizontallyActive();
    if (hflip)
    {
        return *hflip;
    }
    else
    {
        throw std::runtime_error("An error occurred attempting to read Horizontal Flip state.");
    }
}

static bool hflip_set(tofcore::Sensor &sensor, bool active)
{
    py::gil_scoped_release gsr;
    return sensor.setFlipHorizontally(active);
}

static bool vflip_get(tofcore::Sensor &sensor)
{
    py::gil_scoped_release gsr;
    auto vflip = sensor.isFlipVerticallyActive();
    if (vflip)
    {
        return *vflip;
    }
    else
    {
        throw std::runtime_error("An error occurred attempting to read Vertical Flip state.");
    }
}

static bool vflip_set(tofcore::Sensor &sensor, bool active)
{
    py::gil_scoped_release gsr;
    return sensor.setFlipVertically(active);;
}

static std::optional<TofComm::VsmControl_T> get_vsm(tofcore::Sensor &sensor)
{
    py::gil_scoped_release gsr;
    auto vsm = sensor.getVsmSettings();
    if (vsm)
    {
        return vsm;
    }
    else
    {
        throw std::runtime_error("An error occurred attempting to get Vsm Settings.");
    }
}

static std::optional<uint32_t> getSensorVSMMaxNumberOfElements(tofcore::Sensor &sensor)
{
    py::gil_scoped_release gsr;
    auto vsmLimit = sensor.getVsmMaxNumberOfElements();
    if (vsmLimit)
    {
        return vsmLimit;
    }
    else
    {
        throw std::runtime_error("An error occurred attempting to get Vsm Settings Limit.");
    }
}

static bool set_vsm(tofcore::Sensor &sensor, const TofComm::VsmControl_T& vsmControl)
{
    py::gil_scoped_release gsr;
    bool result = sensor.setVsm(vsmControl);
    if (!result)
    {
        throw std::runtime_error("Unable to set Vsm Settings.");
    }
    return result;
}

static auto imuAccelerometerSelfTest(tofcore::Sensor& s)
{
    int result = 0;
    {
        py::gil_scoped_release gsr;
        result = s.imuAccelerometerSelfTest();
    }
    return result;
}

static auto jump_to_bootloader(tofcore::Sensor& s)
{
    s.jumpToBootloader();
}

static auto reset_sensor(tofcore::Sensor& s)
{
    s.jumpToBootloader(0x123);
}

auto durationToDuration(const float time_s)
{
    using namespace std::chrono;
    using fsec = duration<float>;
    return round<nanoseconds>(fsec{time_s});
}

static auto py_find_all_devices(double timeout_sec, std::size_t max_count)
{
    return tofcore::find_all_devices(durationToDuration(timeout_sec), max_count);
}

static bool setBinning(tofcore::Sensor& sensor, const bool vertical, const bool horizontal)
{
    py::gil_scoped_release gsr;
    if ((vertical && !horizontal) || (!vertical && horizontal)) {
        std::cout << "Partial binning is being deprecated. Use set_binning(True) instead for full binning" << std::endl;

    }
    return sensor.setBinning(vertical, horizontal);
}


PYBIND11_MODULE(pytofcore, m) {
    m.doc() = "Sensor object that represents a connect to a TOF depth sensor.";

    m.attr("IPv4Settings") = PyIPv4Settings;
    m.attr("IPv4Endpoint") = PyIPv4Endpoint;
    m.attr("IPv4LogSettings") = PyIPv4LogSettings;

    py::class_<tofcore::Sensor>(m, "Sensor")
        .def(py::init<const std::string&>(), py::arg("uri")=tofcore::DEFAULT_URI)
        .def(py::init<const std::string&, uint32_t>(), py::arg("port_name")=tofcore::DEFAULT_PORT_NAME, py::arg("baud_rate")=tofcore::DEFAULT_BAUD_RATE)
        .def_property_readonly("pixel_rays", &getPixelRays, "Obtain unit vector ray information for all pixels based on the lens information stored on the sensor. Returns a namedtuple with fields x, y, z. Each field is a list of floats of length width x height.")
        .def_property_readonly("lens_info", &getLensInfo, "Obtain Lens information stored on sensor. Returns a namedtuple with fields rowOffset, columnOffset, rowFocalLength, columnFocalLength, undistortionCoeffs, hfov, vfov.")

        .def("get_binning", &tofcore::Sensor::getBinning, "Get binning setting on sensor", py::call_guard<py::gil_scoped_release>())
        .def("get_frame_crc_state", &tofcore::Sensor::getFrameCrcState, "Get state of frame CRC calculation", py::call_guard<py::gil_scoped_release>())
        .def("get_frame_period_and_limits", &getSensorFramePeriodAndLimits, "query the device for the currently configured frame period limits", py::call_guard<py::gil_scoped_release>())
        .def("get_frame_period", &tofcore::Sensor::getFramePeriodMs, "Get target frame period", py::call_guard<py::gil_scoped_release>())
        .def("get_hdr_settings", &tofcore::Sensor::getHdrSettings, "Get the current HDR settings from the sensor", py::call_guard<py::gil_scoped_release>())
        .def("get_imu_data", &getImuData, "Imu data")
        .def("get_imu_accelerometer_available_ranges", &tofcore::Sensor::imuAccelerometerAvailableRangesInGs, "Imu accelerometer available ranges", py::call_guard<py::gil_scoped_release>())
        .def("get_integration_time_and_limits", &getSensorIntegrationTimeAndLimits, "query the device for the currently configured integration time limits", py::call_guard<py::gil_scoped_release>())
        .def("get_integration_time", &getSensorIntegrationTime, "query the device for the currently configured integration time setting", py::call_guard<py::gil_scoped_release>())
        .def("get_min_amplitude_and_limits", &getSensorMinAmplitudeAndLimits, "query the device for the currently configured min amplitude limits", py::call_guard<py::gil_scoped_release>())
        .def("get_min_amplitude", &tofcore::Sensor::getMinAmplitude, "Get minimum amplitude for distance pixel to be treated as good", py::call_guard<py::gil_scoped_release>())
        .def("get_modfreq_and_limits_and_step", &getModulationFrequencyAndLimitsAndStepSize, "query the device for the currently configured modulation frequency limits and step size", py::call_guard<py::gil_scoped_release>())
        .def("get_sensor_info", &getSensorInfo, "Get the sensor version and build info")
        .def("get_sensor_status", &getSensorStatus, "Get the sensor status info")
        .def("get_tof_control_status", &getSensorControlStatus, "Gets the state of the ToF controller")
        .def("get_vled_setting_and_limits", &getSensorVledSettingAndLimits, "Get the setting store in the vled register, as well as the max/min VLED limits in mV.", py::call_guard<py::gil_scoped_release>())
        .def("get_vsm_max_number_of_elements", &getSensorVSMMaxNumberOfElements, "query the device for the maximum number of VSM elements")
        .def("get_vsm", &get_vsm, "Get the current Vector Sequence Mode (VSM) settings")
        .def("imu_accelerometer_self_test", &imuAccelerometerSelfTest, "Imu accelerometer self-test")
        .def("imu_accelerometer_range", static_cast<std::tuple<int8_t, uint8_t>(tofcore::Sensor::*)()>(&tofcore::Sensor::imuAccelerometerRangeInGs), "Get the Imu accelerometer current range", py::call_guard<py::gil_scoped_release>())
        .def("imu_accelerometer_range", static_cast<int8_t (tofcore::Sensor::*)(uint8_t)>(&tofcore::Sensor::imuAccelerometerRangeInGs), "Set the Imu accelerometer range", py::call_guard<py::gil_scoped_release>())
        .def("is_raw_data_sorted", &tofcore::Sensor::isRawDataSorted, "Get raw data sorting setting on sensor", py::call_guard<py::gil_scoped_release>())
        .def("jump_to_bootloader", &jump_to_bootloader, "Activate bootloader mode to flash firmware", py::call_guard<py::gil_scoped_release>())
        .def("reset_sensor", &reset_sensor, "Perform a software reset of the sensor", py::call_guard<py::gil_scoped_release>())
        .def("set_binning", py::overload_cast<bool>(&tofcore::Sensor::setBinning), "Set binning on device", py::arg("binning"), py::call_guard<py::gil_scoped_release>())
        .def("set_binning", py::overload_cast<tofcore::Sensor&, bool, bool>(&setBinning), "Set horizontal and vertical binning settings on sensor. Being deprecated", py::arg("vertical"), py::arg("horizontal"))
        .def("set_frame_crc_state", &tofcore::Sensor::setFrameCrcState, py::arg("state"), "Set state of frame CRC calculation", py::call_guard<py::gil_scoped_release>())
        .def("set_frame_period", &tofcore::Sensor::setFramePeriodMs, py::arg("period_in_ms"), "Set target frame period", py::call_guard<py::gil_scoped_release>())
        .def("set_integration_time", &tofcore::Sensor::setIntegrationTime, "Set the integration time parameter on the sensor", py::arg("timeUs"), py::call_guard<py::gil_scoped_release>())
        .def("set_integration_times", &tofcore::Sensor::setIntegrationTimes, "Set all integration time parameters on the sensor", py::arg("low"), py::arg("mid"), py::arg("high"), py::call_guard<py::gil_scoped_release>())
        .def("set_min_amplitude", &tofcore::Sensor::setMinAmplitude, py::arg("min_amplitude"), "Set minimum amplitude for distance pixel to be treated as good", py::call_guard<py::gil_scoped_release>())
        .def("set_modulation", &modulation_set_deprecated, "DEPRECATED (use modulation_frequency property) Set the modulation frequency to use during TOF measurements", py::arg("index"), py::arg("channel"))
        .def("set_offset", &tofcore::Sensor::setOffset, py::arg("offset"), "Apply milimeter offest to very distance pixel returned by the sensor", py::call_guard<py::gil_scoped_release>())
        .def("set_vsm", &set_vsm, "Set the current Vector Sequence Mode (VSM) settings")
        .def("set_hdr", &tofcore::Sensor::setHdr, py::arg("enable"), py::arg("use_spatial")=false, "Enable or disable High Dynamic Range mode", py::call_guard<py::gil_scoped_release>())
        .def("sort_raw_data", &tofcore::Sensor::sortRawData, py::arg("sort_it"), "Enable or disable on-sensor sorting of raw data", py::call_guard<py::gil_scoped_release>())
        .def("stop_stream", &tofcore::Sensor::stopStream, "Command the sensor to stop streaming", py::call_guard<py::gil_scoped_release>())
        .def("storeSettings", &tofcore::Sensor::storeSettings, "Store the sensor's settings to persistent memory", py::call_guard<py::gil_scoped_release>())
        .def("stream_dcs_ambient", &tofcore::Sensor::streamDCSAmbient, "Command the sensor to stream DCS and Ambient frames. Ambient frames are of type Frame::DataType::GRAYSCALE", py::call_guard<py::gil_scoped_release>())
        .def("stream_dcs_diff_ambient", &tofcore::Sensor::streamDCSDiffAmbient, "Command the sensor to stream DCS_DIFF and Ambient frame.", py::call_guard<py::gil_scoped_release>())
        .def("stream_dcs", &tofcore::Sensor::streamDCS, "Command the sensor to stream DCS frames", py::call_guard<py::gil_scoped_release>())
        .def("stream_distance_amplitude", &tofcore::Sensor::streamDistanceAmplitude, "Command the sensor to stream distance and amplitude frames", py::call_guard<py::gil_scoped_release>())
        .def("stream_distance", &tofcore::Sensor::streamDistance, "Command the sensor to stream distance frames", py::call_guard<py::gil_scoped_release>())
        .def("subscribe_measurement", &subscribeMeasurement, "Set a function object to be called when new measurement data is received", py::arg("callback"))

        .def_property("hflip", &hflip_get, &hflip_set, "State of the image horizontal flip option (default False)")
        .def_property("ip_measurement_endpoint", &getIPMeasurementEndpoint, &setIPMeasurementEndpoint, "The IP address and port measurement data is set to")
        .def_property("ipv4_settings", &getIPv4Settings, &setIPv4Settings, "Set the IPv4 address, mask, and gateway")
        .def_property("ipv4_log_settings", &getIPv4LogSettings, &setIPv4LogSettings, "Set the IPv4 address and port to which log output is sent")
        .def_property("modulation_frequency", &modulation_get, &modulation_set, "LED Modulation Frequency in kHz (default 24000)")
        .def_property("sensor_location", &getSensorLocation, &setSensorLocation, "The sensor's location")
        .def_property("sensor_name", &getSensorName, &setSensorName, "The sensor's name")
        .def_property("vflip", &vflip_get, &vflip_set, "State of the image vertical flip option (default False)")

        .def_property_readonly_static("DEFAULT_URI", [](py::object /* self */){return tofcore::DEFAULT_URI;})
        .def_property_readonly_static("DEFAULT_PORT_NAME", [](py::object /* self */){return tofcore::DEFAULT_PORT_NAME;})
        .def_property_readonly_static("DEFAULT_BAUD_RATE", [](py::object /* self */){return tofcore::DEFAULT_BAUD_RATE;});


    py::class_<tofcore::Measurement_T, std::shared_ptr<tofcore::Measurement_T>> measurement(m, "Measurement");
    //Note: there is intentionally no init function for measurement, currently these must be produced by receiving data from a Sensor class
    measurement
        .def_property_readonly("ambient_data", &get_ambient_view, "obtain memoryview of the ambient frame in the measurement")
        .def_property_readonly("amplitude_data", &get_amplitude_view, "obtain memoryview of the amplitude frame in the measurement")
        .def_property_readonly("crc_errors", &tofcore::Measurement_T::crc_errors, "indicates whether CRC errors were observed")
        .def_property_readonly("data_type", &tofcore::Measurement_T::type, "the type of the measurement data")
        .def_property_readonly("dcs_data", &get_dcs_view, DCS_DATA_DOCSTRING)
        .def_property_readonly("dcs_diff_data", &get_dcs_diff_view, DCS_DIFF_DATA_DOCSTRING)
        .def_property_readonly("distance_data", &get_distance_view, "obtain memoryview of the distance frame in the measurement")
        .def_property_readonly("dll_settings", &get_dll_settings, "get the dll settings used during capture as a named tuple of (enabled, coarseStep, fineStep, finestStep)")
        .def_property_readonly("height", &tofcore::Measurement_T::height, "height in pixels of measurement data")
        .def_property_readonly("horizontal_binning", &tofcore::Measurement_T::horizontal_binning, "get horizontal binning setting used during capture, 0 means no binning, values above 1 indicate the amount of subsampling")
        .def_property_readonly("illuminator_info", &get_illuminator_info, ILLUMINATOR_INFO_DOCSTRING)
        .def_property_readonly("integration_time", &tofcore::Measurement_T::integration_time, "get integration time setting during capture (uint16_t uS)")
        .def_property_readonly("meta_data", &get_meta_data_view, "obtain memoryview of the raw block of meta-data associated with the measurement (useful for custom decoding of data not otherwise available via the API)")
        .def_property_readonly("modulation_frequency", &tofcore::Measurement_T::modulation_frequency, "get modulation frequency (Hz) setting during capture")
        .def_property_readonly("pixel_size", &tofcore::Measurement_T::pixel_size, "size in bytes of each pixel")
        .def_property_readonly("sensor_temperatures", &tofcore::Measurement_T::sensor_temperatures, "get imaging sensor temperature data")
        .def_property_readonly("timestamp", &tofcore::Measurement_T::frame_timestamp, "get the frame timestamp (in ms)")
        .def_property_readonly("vertical_binning", &tofcore::Measurement_T::vertical_binning, "get vertical binning setting used during capture, 0 means no binning, values above 1 indicate the amount of subsampling")
        .def_property_readonly("width", &tofcore::Measurement_T::width, "width in pixels of measurement data")
        ;


    py::enum_<tofcore::SensorControlStatus> tof_control_enum(m, "SensorControlStatus");
    tof_control_enum
        .value("IDLE", tofcore::SensorControlStatus::IDLE)
        .value("CAPTURE", tofcore::SensorControlStatus::CAPTURE)
        .value("SEND", tofcore::SensorControlStatus::SEND)
        .value("STREAM", tofcore::SensorControlStatus::STREAM)
        .value("OVERTEMPERATURE", tofcore::SensorControlStatus::OVERTEMPERATURE)
        .value("ERROR", tofcore::SensorControlStatus::ERROR)
        .export_values();

    py::enum_<tofcore::Measurement_T::DataType> data_type_enum(measurement, "DataType");
    data_type_enum
        .value("UNKNOWN", tofcore::Measurement_T::DataType::UNKNOWN)
        .value("DISTANCE_AMPLITUDE", tofcore::Measurement_T::DataType::DISTANCE_AMPLITUDE)
        .value("DISTANCE", tofcore::Measurement_T::DataType::DISTANCE)
        .value("AMPLITUDE", tofcore::Measurement_T::DataType::AMPLITUDE)
        .value("GRAYSCALE", tofcore::Measurement_T::DataType::GRAYSCALE)
        .value("DCS", tofcore::Measurement_T::DataType::DCS)
        .value("AMBIENT", tofcore::Measurement_T::DataType::AMBIENT)
        .value("DCS_DIFF_AMBIENT", tofcore::Measurement_T::DataType::DCS_DIFF_AMBIENT)
        .export_values();

    py::class_<TofComm::VsmElement_T>(m, "VsmElement")
        .def(py::init<>())
        .def_readwrite("integration_time", &TofComm::VsmElement_T::m_integrationTimeUs)
        .def_readwrite("modulation_frequency", &TofComm::VsmElement_T::m_modulationFreqKhz)
        ;

    py::class_<TofComm::VsmControl_T>(m, "VsmControl")
        .def(py::init<>())
        .def_readwrite("vsm_flags", &TofComm::VsmControl_T::m_vsmFlags)
        .def_readwrite("vsm_index", &TofComm::VsmControl_T::m_vsmIndex)
        .def_property("elements",
            [](const TofComm::VsmControl_T &obj) {
                std::vector<TofComm::VsmElement_T> arr(obj.m_numberOfElements);
                for (size_t i = 0; i < obj.m_numberOfElements; ++i) {
                    arr[i] = obj.m_elements[i];
                }
                return arr;
            },
            [](TofComm::VsmControl_T &obj, const std::vector<TofComm::VsmElement_T> &arr) {
                if (arr.size() > TofComm::VSM_MAX_NUMBER_OF_ELEMENTS) {
                    throw std::runtime_error("Array of Vsm elements is the incorrect size");
                }
                for (size_t i = 0; i < arr.size(); ++i) {
                    obj.m_elements[i] = arr[i];
                }
                obj.m_numberOfElements = arr.size();
            }
        )
        ;

    py::enum_<TofComm::HdrMode_e> hdr_mode_enum(m, "HdrModeEnum");
    hdr_mode_enum
        .value("UNKOWN", TofComm::HdrMode_e::UNKOWN)
        .value("TEMPORAL", TofComm::HdrMode_e::TEMPORAL)
        .value("SPATIAL", TofComm::HdrMode_e::SPATIAL)
        .export_values();

    py::class_<TofComm::HdrSettings_T>(m, "HdrSettings")
        .def(py::init<>())
        .def_readonly("enabled", &TofComm::HdrSettings_T::enabled)
        .def_readonly("mode", &TofComm::HdrSettings_T::mode)
        ;

    m.attr("DataType") = data_type_enum;
    m.attr("SensorControlStatus") = tof_control_enum;

    #ifdef VERSION_INFO
        m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
    #else
        m.attr("__version__") = "dev";
    #endif

    py::class_<tofcore::device_info_t>(m, "DeviceInfo")
        .def_readonly("connector_uri", &tofcore::device_info_t::connector_uri)
        .def_readonly("serial_num", &tofcore::device_info_t::serial_num)
        .def_readonly("model", &tofcore::device_info_t::model)
        .def("__repr__",
            [](const tofcore::device_info_t &d) {
                return "pytofcore.DeviceInfo(connector_uri:'" + d.connector_uri
                       + "', serial_num:'" + d.serial_num + "', model:'" + d.model + "')";
            });

    m.def("find_all_devices", &py_find_all_devices, py::arg("wait_time")=5, py::arg("max_count")=std::numeric_limits<std::size_t>::max(), "Get a list of all connected devices");

}
