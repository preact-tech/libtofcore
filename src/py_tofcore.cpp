#include "tof_sensor.hpp"

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


namespace py = pybind11;

/// @brief Utility class to allow overriding some methods from tofcore::Sensor 
///  with Python specific functionality
class PySensor : public tofcore::Sensor
{
public:
    using Sensor::Sensor;

    /// @brief Wrap python callback with lambda that will catch exceptions and handle them safely
    /// @param py_callback client callback functor
    void subscribeMeasurement(on_measurement_ready_t py_callback)
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
        tofcore::Sensor::subscribeMeasurement(f);
    }

   auto getSoftwareVersion()
   {
       //Use static and a lambda to create the softwareVersion namedtuple type only once.
       static auto softwareVersion_type = []() {
           auto namedTuple_attr = pybind11::module::import("collections").attr("namedtuple");
           py::list fields;
           fields.append("softwareVersion");
           return namedTuple_attr("version", fields);
       }();

       std::string softwareVersion;

       if (!tofcore::Sensor::getSoftwareVersion(softwareVersion))
       {
           throw std::runtime_error("An error occcured trying to get firmware version");
       }

       return softwareVersion_type(softwareVersion);
   }

    auto getChipInformation()
    {
        //Use static and a lambda to create the ChipInfo namedtuple type only once.
        static auto ChipInfo_type = []() {
            auto namedTuple_attr = pybind11::module::import("collections").attr("namedtuple");
            py::list fields;
            fields.append("wafer_id");
            fields.append("chip_id");
            return namedTuple_attr("ChipInfo", fields);
        }();

        uint16_t waferId, chipId;
        if (!tofcore::Sensor::getChipInformation(waferId, chipId)) 
        {
            throw std::runtime_error("An error occurred while trying to get chip information.");
        }

        return ChipInfo_type(waferId, chipId);
    }

    auto getAccelerometerData() 
    {

        //Use static and a lambda to create the AccelerometerData namedtuple type only once.
        static auto AccelerometerData_type = []() {
            auto namedTuple_attr = pybind11::module::import("collections").attr("namedtuple");
            py::list fields;
            fields.append("x");
            fields.append("y");
            fields.append("z");
            fields.append("g_range");
            return namedTuple_attr("AccelerometerData", fields);
        }();

        int16_t x, y, z;
        uint8_t g_range;
        if (!tofcore::Sensor::getAccelerometerData(x, y, z, g_range)) 
        {
            throw std::runtime_error("An error occured while getting accelerometer data");
        }

        return AccelerometerData_type(x, y, z, g_range);
    }

    auto getPixelRays()
    {
        //Use static and a lambda to create the AccelerometerData namedtuple type only once.
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

        if(!tofcore::Sensor::getLensInfo(rays_x, rays_y, rays_z))
        {
            throw std::runtime_error("An error occured while getting pixel ray information");
        }

        return PixelRays_type(rays_x, rays_y, rays_z);
    }

    auto getSensorInfo()
    {

        //Use static and a lambda to create the VersionData namedtuple type only once. 
        static auto VersionData_type = []() {
            auto namedTuple_attr = pybind11::module::import("collections").attr("namedtuple");
            py::list fields;
            fields.append("serialNumber");
            fields.append("modelNumber");
            fields.append("modelName");

            fields.append("softwareId");
            fields.append("softwareVersion");

            // Mojave platform only
            fields.append("cpuVersion");
            fields.append("illuminatorSwVersion");
            fields.append("illuminatorSwId");
            fields.append("illuminatorHwCfg");
            fields.append("backpackModule");

            return namedTuple_attr("VersionData", fields);
        }();

        TofComm::versionData_t versionData;

        if (!tofcore::Sensor::getSensorInfo(versionData))
        {
            throw std::runtime_error("An error occcured trying to read sensor version info");
        }

        return VersionData_type(versionData.m_serialNumber, 
                                versionData.m_modelNumber, 
                                versionData.m_modelName, 
                                versionData.m_softwareSourceID, 
                                versionData.m_softwareVersion,
                                versionData.m_cpuVersion, 
                                versionData.m_illuminatorSwVersion,
                                versionData.m_illuminatorSwSourceId,
                                versionData.m_illuminatorHwCfg,
                                (uint8_t)versionData.m_backpackModule);
    }

};


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


/// @brief Helper function to obtain a memoryview of the grayscale data in
///   a Measurement object
static auto get_grayscale_view(const tofcore::Measurement_T &m)
{
    const auto view = m.grayscale();
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

PYBIND11_MODULE(pytofcore, m) {
    m.doc() = "Sensor object that represents a connect to a TOF depth sensor.";

    py::class_<PySensor>(m, "Sensor")
        .def(py::init<uint16_t, const std::string&, uint32_t>(), py::arg("protocol_version")=tofcore::DEFAULT_PROTOCOL_VERSION, py::arg("port_name")=tofcore::DEFAULT_PORT_NAME, py::arg("baud_rate")=tofcore::DEFAULT_BAUD_RATE)
        .def_property_readonly("get_software_version", &PySensor::getSoftwareVersion, "Obtain the software build version string", py::call_guard<py::gil_scoped_release>())
        .def_property_readonly("chip_info", &PySensor::getChipInformation, "Obtain chip info. Returns namedtuple with fields wafer_id and chip_id", py::call_guard<py::gil_scoped_release>())
        .def_property_readonly("accelerometer_data", &PySensor::getAccelerometerData, "Obtain acceleromter info. Each call turns a new sample from the accelerometer. The same is a namedtuple with fields x, y, z, range_g", py::call_guard<py::gil_scoped_release>())
        .def_property_readonly("pixel_rays", &PySensor::getPixelRays, "Obtain unit vector ray information for all pixels based on the lens information stored on the sensor. Returns a namedtuple with fields x, y, z. Each field is a list of floats of length width x height.", py::call_guard<py::gil_scoped_release>())
        .def("stop_stream", &PySensor::stopStream, "Command the sensor to stop streaming", py::call_guard<py::gil_scoped_release>())
        .def("stream_dcs", &PySensor::streamDCS, "Command the sensor to stream DCS frames", py::call_guard<py::gil_scoped_release>())
        .def("stream_dcs_ambient", &PySensor::streamDCSAmbient, "Command the sensor to stream DCS and Ambient frames. Ambient frames are of type Frame::DataType::GRAYSCALE", py::call_guard<py::gil_scoped_release>())
        .def("stream_grayscale", &PySensor::streamGrayscale, "Command the sensor to stream grayscale frames", py::call_guard<py::gil_scoped_release>())
        .def("stream_distance", &PySensor::streamDistance, "Command the sensor to stream distance frames", py::call_guard<py::gil_scoped_release>())
        .def("stream_distance_amplitude", &PySensor::streamDistanceAmplitude, "Command the sensor to stream distance and amplitude frames", py::call_guard<py::gil_scoped_release>())
        .def("set_offset", &PySensor::setOffset, py::arg("offset"), "Apply milimeter offest to very distance pixel returned by the sensor", py::call_guard<py::gil_scoped_release>())
        .def("set_min_amplitude", &PySensor::setMinAmplitude, py::arg("min_amplitude"), "Set minimum amplitude for distance pixel to be treated as good", py::call_guard<py::gil_scoped_release>())
        .def("set_binning", &PySensor::setBinning, "Set horizontal and vertical binning settings on sensor", py::arg("vertical"), py::arg("horizontal"), py::call_guard<py::gil_scoped_release>())
        .def("set_roi", &PySensor::setRoi, "Set region of interest pixel area on the sensor", py::arg("x0"), py::arg("y0"), py::arg("x1"), py::arg("y1"), py::call_guard<py::gil_scoped_release>())
        .def("set_integration_time", &PySensor::setIntegrationTime, "Set the integration time parameters on the sensor", py::arg("low"), py::arg("mid"), py::arg("high"), py::arg("gray"), py::call_guard<py::gil_scoped_release>())
        .def("set_hdr_mode", &PySensor::setHDRMode, "Set the High Dynamic Range modeon the sensor", py::arg("mode"), py::call_guard<py::gil_scoped_release>())
        .def("set_modulation", &PySensor::setModulation, "Set the modulation frequency to use during TOF measurements", py::arg("index"), py::arg("channel"), py::call_guard<py::gil_scoped_release>())
        .def("set_filter", &PySensor::setFilter, "Configure filter applied by sensor on data returned", py::call_guard<py::gil_scoped_release>())
        .def("subscribe_measurement", &PySensor::subscribeMeasurement, "Set a function object to be called when new measurement data is received", py::arg("callback"))
        .def_property_readonly_static("DEFAULT_PORT_NAME", [](py::object /* self */){return tofcore::DEFAULT_PORT_NAME;})
        .def_property_readonly_static("DEFAULT_BAUD_RATE", [](py::object /* self */){return tofcore::DEFAULT_BAUD_RATE;})
        .def_property_readonly_static("DEFAULT_PROTOCOL_VERSION", [](py::object /* self */){return tofcore::DEFAULT_PROTOCOL_VERSION;});


    py::class_<tofcore::Measurement_T, std::shared_ptr<tofcore::Measurement_T>> measurement(m, "Measurement");
    //Note: there is intentionally no init function for measurement, currently these must be produced by receiving data from a Sensor class
    measurement
        .def_property_readonly("width", &tofcore::Measurement_T::width, "width in pixels of measuremnt data")
        .def_property_readonly("height", &tofcore::Measurement_T::height, "height in pixels of measuremnt data")
        .def_property_readonly("pixel_size", &tofcore::Measurement_T::pixel_size, "size in bytes of each pixel")
        .def_property_readonly("data_type", &tofcore::Measurement_T::type, "the type of the measurement data")
        .def_property_readonly("distance_data", &get_distance_view, "obtain memoryview of the distance frame in the measurement")
        .def_property_readonly("amplitude_data", &get_amplitude_view, "obtain memoryview of the amplitude frame in the measurement")
        .def_property_readonly("grayscale_data", &get_grayscale_view, "obtain memoryview of the grayscale (aka ambient) frame in the measurement")
        .def_property_readonly("dcs_data", &get_dcs_view, DCS_DATA_DOCSTRING)
        .def_property_readonly("meta_data", &get_meta_data_view, "obtain memoryview of the raw block of meta-data associated with the measurement (useful for custom decoding of data not otherwise available via the API)")
        .def_property_readonly("sensor_temperatures", &tofcore::Measurement_T::sensor_temperatures, "get imaging sensor temperature data")
        .def_property_readonly("integration_times", &tofcore::Measurement_T::integration_times, "get integration time settings during capture [int0, int1, int2, grayscale], values actually used depend on type of measuremnt and additional options")
        .def_property_readonly("modulation_frequencies", &tofcore::Measurement_T::modulation_frequencies, "get modulation frequency (Hz) settings during capture")
        .def_property_readonly("horizontal_binning", &tofcore::Measurement_T::horizontal_binning, "get horizontal binning setting used during capture, 0 means no binning, values above 1 indicate the amount of subsampling")
        .def_property_readonly("vertical_binning", &tofcore::Measurement_T::vertical_binning, "get vertical binning setting used during capture, 0 means no binning, values above 1 indicate the amount of subsampling")
        .def_property_readonly("dll_settings", &get_dll_settings, "get the dll settings used during capture as a named tuple of (enabled, coarseStep, fineStep, finestStep)")
        ;

    py::enum_<tofcore::Measurement_T::DataType>(measurement, "DataType")
        .value("UNKNOWN", tofcore::Measurement_T::DataType::UNKNOWN)
        .value("DISTANCE_AMPLITUDE", tofcore::Measurement_T::DataType::DISTANCE_AMPLITUDE)
        .value("DISTANCE", tofcore::Measurement_T::DataType::DISTANCE)
        .value("AMPLITUDE", tofcore::Measurement_T::DataType::AMPLITUDE)
        .value("GRAYSCALE", tofcore::Measurement_T::DataType::GRAYSCALE)
        .value("DCS", tofcore::Measurement_T::DataType::DCS)
        .export_values();

    #ifdef VERSION_INFO
        m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
    #else
        m.attr("__version__") = "dev";
    #endif
}
