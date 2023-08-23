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
  "DCS pixels are encoded as 16 bit signed little endian values with upto 14-bits\n"
  "of precision possible but hardware typically limits to 12-bits\n"
  "Bit 14 is used to encode a saturation flag such that a saturated pixels bit-14 will"
  "be the opposite of the sign bit.";

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

static auto getIPv4Settings(tofcore::Sensor& s)
{
    //Use static and a lambda to create the IPv4Settings namedtuple type only once.
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
        throw std::runtime_error("An error occured while getting IPv4 settings");
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
        throw std::runtime_error("An error occcured setting sequencer version");
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
                         result->m_columnFocalLength, result->m_undistortionCoeffs);
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
        throw std::runtime_error("An error occcured trying to read sensor version info");
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
                            versionData.m_cpuVersion,
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
        throw std::runtime_error("An error occcured setting sensor location");
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
        throw std::runtime_error("An error occcured setting sensor name");
    }
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
        throw std::runtime_error("An error occcured trying to read sensor status info");
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
static auto getSensorIntegrationTimes(tofcore::Sensor& s)
{
    auto integrationTimes = s.getIntegrationTimes();
    if (!integrationTimes)
    {
        throw std::runtime_error("An error occured while getting integration times");
    }

    return integrationTimes;
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


/// @brief Helper function to obtain a memoryview of the grayscale data in
///   a Measurement object
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
        throw std::runtime_error("An error occcured attempting to read modulation frequency state.");
    }
}

static bool modulation_set(tofcore::Sensor &sensor, uint16_t modFreqkHz)
{
    py::gil_scoped_release gsr;
    return sensor.setModulation(modFreqkHz);
}


/// @brief Depricated version of modulation frequency set.
/// This is just here to support the old method of setting modulation frequency using enumerated value
/// where index values represented specific frequencies: 0 == 12mHz, 1 == 24mHz, 2 = 6mHz
/// @return boolean indicating success or false
static bool modulation_set_depricated(tofcore::Sensor &sensor, int index, int channel)
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
        throw std::runtime_error("An error occcured attempting to read Horizontal Flip state.");
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
        throw std::runtime_error("An error occcured attempting to read Vertical Flip state.");
    }
}

static bool vflip_set(tofcore::Sensor &sensor, bool active)
{
    py::gil_scoped_release gsr;
    return sensor.setFlipVertically(active);;
}

static auto jump_to_bootloader(tofcore::Sensor& s)
{
    s.jumpToBootloader();
}


PYBIND11_MODULE(pytofcore, m) {
    m.doc() = "Sensor object that represents a connect to a TOF depth sensor.";

    m.attr("IPv4Settings") = PyIPv4Settings;

    py::class_<tofcore::Sensor>(m, "Sensor")
        .def(py::init<const std::string&>(), py::arg("uri")=tofcore::DEFAULT_URI)
        .def(py::init<uint16_t, const std::string&, uint32_t>(), py::arg("protocol_version")=tofcore::DEFAULT_PROTOCOL_VERSION, py::arg("port_name")=tofcore::DEFAULT_PORT_NAME, py::arg("baud_rate")=tofcore::DEFAULT_BAUD_RATE)
        .def_property_readonly("pixel_rays", &getPixelRays, "Obtain unit vector ray information for all pixels based on the lens information stored on the sensor. Returns a namedtuple with fields x, y, z. Each field is a list of floats of length width x height.")
        .def_property_readonly("lens_info", &getLensInfo, "Obtain Lens information stored on sensor. Returns a namedtuple with fields rowOffset, columnOffset, rowFocalLength, columnFocalLength, undistortionCoeffs.")
        .def("stop_stream", &tofcore::Sensor::stopStream, "Command the sensor to stop streaming", py::call_guard<py::gil_scoped_release>())
        .def("stream_dcs", &tofcore::Sensor::streamDCS, "Command the sensor to stream DCS frames", py::call_guard<py::gil_scoped_release>())
        .def("stream_dcs_ambient", &tofcore::Sensor::streamDCSAmbient, "Command the sensor to stream DCS and Ambient frames. Ambient frames are of type Frame::DataType::GRAYSCALE", py::call_guard<py::gil_scoped_release>())
        .def("stream_distance", &tofcore::Sensor::streamDistance, "Command the sensor to stream distance frames", py::call_guard<py::gil_scoped_release>())
        .def("stream_distance_amplitude", &tofcore::Sensor::streamDistanceAmplitude, "Command the sensor to stream distance and amplitude frames", py::call_guard<py::gil_scoped_release>())
        .def("set_offset", &tofcore::Sensor::setOffset, py::arg("offset"), "Apply milimeter offest to very distance pixel returned by the sensor", py::call_guard<py::gil_scoped_release>())
        .def("set_min_amplitude", &tofcore::Sensor::setMinAmplitude, py::arg("min_amplitude"), "Set minimum amplitude for distance pixel to be treated as good", py::call_guard<py::gil_scoped_release>())
        .def("set_binning", &tofcore::Sensor::setBinning, "Set horizontal and vertical binning settings on sensor", py::arg("vertical"), py::arg("horizontal"), py::call_guard<py::gil_scoped_release>())
        .def("set_integration_times", &tofcore::Sensor::setIntegrationTimes, "Set all integration time parameters on the sensor", py::arg("low"), py::arg("mid"), py::arg("high"), py::call_guard<py::gil_scoped_release>())
        .def("get_integration_times", &getSensorIntegrationTimes, "query the device for the currently configured integration time settings", py::call_guard<py::gil_scoped_release>())
        .def("set_hdr_mode", &tofcore::Sensor::setHDRMode, "Set the High Dynamic Range mode on the sensor", py::arg("mode"), py::call_guard<py::gil_scoped_release>())
        .def("subscribe_measurement", &subscribeMeasurement, "Set a function object to be called when new measurement data is received", py::arg("callback"))
        .def("get_sensor_info", &getSensorInfo, "Get the sensor version and build info")
        .def("get_sensor_status", &getSensorStatus, "Get the sensor status info")
        .def("jump_to_bootloader", &jump_to_bootloader, "Activate bootloader mode to flash firmware", py::call_guard<py::gil_scoped_release>())
        .def("storeSettings", &tofcore::Sensor::storeSettings, "Store the sensor's settings to persistent memory", py::call_guard<py::gil_scoped_release>())
        .def("set_modulation", &modulation_set_depricated, "DEPRICATED (use modulation_frequency property) Set the modulation frequency to use during TOF measurements", py::arg("index"), py::arg("channel"))
        .def_property("hflip", &hflip_get, &hflip_set, "State of the image horizontal flip option (default False)")
        .def_property("vflip", &vflip_get, &vflip_set, "State of the image vertical flip option (default False)")
        .def_property("modulation_frequency", &modulation_get, &modulation_set, "LED Modulation Frequency in kHz (default 24000)")        
        .def_property("ipv4_settings", &getIPv4Settings, &setIPv4Settings, "Set the IPv4 address, mask, and gateway")
        .def_property("sensor_location", &getSensorLocation, &setSensorLocation, "The sensor's location")
        .def_property("sensor_name", &getSensorName, &setSensorName, "The sensor's name")

        .def_property_readonly_static("DEFAULT_URI", [](py::object /* self */){return tofcore::DEFAULT_URI;})
        .def_property_readonly_static("DEFAULT_PORT_NAME", [](py::object /* self */){return tofcore::DEFAULT_PORT_NAME;})
        .def_property_readonly_static("DEFAULT_BAUD_RATE", [](py::object /* self */){return tofcore::DEFAULT_BAUD_RATE;})
        .def_property_readonly_static("DEFAULT_PROTOCOL_VERSION", [](py::object /* self */){return tofcore::DEFAULT_PROTOCOL_VERSION;});


    py::class_<tofcore::Measurement_T, std::shared_ptr<tofcore::Measurement_T>> measurement(m, "Measurement");
    //Note: there is intentionally no init function for measurement, currently these must be produced by receiving data from a Sensor class
    measurement
        .def_property_readonly("width", &tofcore::Measurement_T::width, "width in pixels of measurement data")
        .def_property_readonly("height", &tofcore::Measurement_T::height, "height in pixels of measurement data")
        .def_property_readonly("pixel_size", &tofcore::Measurement_T::pixel_size, "size in bytes of each pixel")
        .def_property_readonly("data_type", &tofcore::Measurement_T::type, "the type of the measurement data")
        .def_property_readonly("distance_data", &get_distance_view, "obtain memoryview of the distance frame in the measurement")
        .def_property_readonly("amplitude_data", &get_amplitude_view, "obtain memoryview of the amplitude frame in the measurement")
        .def_property_readonly("ambient_data", &get_ambient_view, "obtain memoryview of the ambient frame in the measurement")
        .def_property_readonly("dcs_data", &get_dcs_view, DCS_DATA_DOCSTRING)
        .def_property_readonly("meta_data", &get_meta_data_view, "obtain memoryview of the raw block of meta-data associated with the measurement (useful for custom decoding of data not otherwise available via the API)")
        .def_property_readonly("sensor_temperatures", &tofcore::Measurement_T::sensor_temperatures, "get imaging sensor temperature data")
        .def_property_readonly("integration_times", &tofcore::Measurement_T::integration_times, "get integration time settings during capture [int0, int1, int2], values actually used depend on type of measurement and additional options")
        .def_property_readonly("modulation_frequencies", &tofcore::Measurement_T::modulation_frequencies, "get modulation frequency (Hz) settings during capture")
        .def_property_readonly("horizontal_binning", &tofcore::Measurement_T::horizontal_binning, "get horizontal binning setting used during capture, 0 means no binning, values above 1 indicate the amount of subsampling")
        .def_property_readonly("vertical_binning", &tofcore::Measurement_T::vertical_binning, "get vertical binning setting used during capture, 0 means no binning, values above 1 indicate the amount of subsampling")
        .def_property_readonly("dll_settings", &get_dll_settings, "get the dll settings used during capture as a named tuple of (enabled, coarseStep, fineStep, finestStep)")
        .def_property_readonly("illuminator_info", &get_illuminator_info, ILLUMINATOR_INFO_DOCSTRING)
        ;


    py::enum_<tofcore::Measurement_T::DataType> data_type_enum(measurement, "DataType");
    data_type_enum
        .value("UNKNOWN", tofcore::Measurement_T::DataType::UNKNOWN)
        .value("DISTANCE_AMPLITUDE", tofcore::Measurement_T::DataType::DISTANCE_AMPLITUDE)
        .value("DISTANCE", tofcore::Measurement_T::DataType::DISTANCE)
        .value("AMPLITUDE", tofcore::Measurement_T::DataType::AMPLITUDE)
        .value("GRAYSCALE", tofcore::Measurement_T::DataType::GRAYSCALE)
        .value("DCS", tofcore::Measurement_T::DataType::DCS)
        .value("AMBIENT", tofcore::Measurement_T::DataType::AMBIENT)
        .export_values();

    m.attr("DataType") = data_type_enum;

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

    m.def("find_all_devices", &tofcore::find_all_devices, "Get a list of all connected devices");

}
