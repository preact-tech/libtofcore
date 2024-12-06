#ifndef __TOFCORE_MEASUREMENT_T_H__
#define __TOFCORE_MEASUREMENT_T_H__
/**
 * @file Measurement_T.hpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * API for libtofcore control
 */

#include "CommandTypes.hpp"
#include "MetaDataTypes.hpp"
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

namespace tofcore
{

/// @brief  Simple BufferView class that provides a non-owning view
///         into a contiguous chunk of memory
/// @tparam T the data type to interpret the memory as
template<typename T>
struct BufferView : public std::tuple<const T*, std::size_t>
{
    using ptr_type = const T*;

    /// @brief Construct a view on some data with size l
    BufferView(ptr_type d, std::size_t l) : 
        std::tuple<const T*, std::size_t>(d, l)
    {}

    /// @brief Return pointer to the start of the view
    ptr_type data() const
    {
        return std::get<0>(*this);
    }

    /// @brief Return the size (in terms of T) of the view
    std::size_t size() const
    {
        return std::get<1>(*this);
    }

    /// @brief Iterator to the start of the view
    ptr_type begin() const
    {
        return data();
    }

    /// @brief Iterator one past the end of the view
    ptr_type end() const
    {
        return data()+size();
    }
};


/// @brief Interface class for managaging and decoding measurement data from a ToF sensor
/// 
/// A Meaurement instance typically contains one or more frames of pixel data 
/// (e.g. 4 DCS frames, distance & amplitude frames) plus some amount of meta-data 
/// associated with the pixel data.
class Measurement_T
{
public:
    using byte_t = std::byte;
    
    enum class DataType:int16_t
    {
        UNKNOWN = -1, 
        DISTANCE_AMPLITUDE = 0,
        DISTANCE = 1,
        AMPLITUDE = 2,
        GRAYSCALE = 3,
        DCS = 4,
        AMBIENT = 5,
        DCS_DIFF_AMBIENT = 6
    };

    virtual ~Measurement_T() = default;

    /// @brief Obtain a view (in raw bytes) of the pixel_buffer for the measurement
    virtual BufferView<byte_t> pixel_buffer() const = 0;

    /// @brief Obtain a view of one of the dcs frames from a DCS measurement
    /// @param dcs_index index of DCS frame to return 0-3
    /// @return view of requested DCS frame data
    /// @return empty view if index out of range or not a DCS measurement
    virtual BufferView<int16_t> dcs(int dcs_index) const = 0;

    /// @brief Obtain a view of one of either the (DCS2-DCS0)
    ///        or (DCS3-DCS1) frames from a DCS Diff + Ambient measurement
    /// @param index 0 for (DCS2-DCS0), 1 for (DCS3-DCS1)
    /// @return view of requested frame data
    /// @return empty view if index out of range or not a DCS Diff + Ambient measurement
    virtual BufferView<int16_t> dcs_diff(int index) const = 0;

    /// @brief Obtain a view of the distance data from a distance measurement
    /// @return view of distance data
    /// @return empty view if not a distance measurement
    virtual BufferView<uint16_t> distance() const = 0;

    /// @brief Obtain a view of the amplitude data from a amplitude measurement
    /// @return view of amplitude data
    /// @return empty view if not a amplitude measurement
    virtual BufferView<uint16_t> amplitude() const = 0;

    /// @brief Obtain a view of the grayscale (ambient) data from a grayscale
    ///        measurement or from a DCS Diff + Ambient measurement
    /// @return view of grayscale data
    /// @return empty view if not a grayscale measurement
    virtual BufferView<int16_t> ambient() const = 0;

    /// @brief Obtain a view of the meta_data associated with the measurement
    /// @return view of meta data
    virtual BufferView<byte_t> meta_data() const = 0;

    /// @brief Width, in pixels, of a frame of data in this measurement
    virtual int width() const = 0;

    /// @brief Height, in pixels, of a frame of data in this measurement
    virtual int height() const = 0;

    /// @brief X location, in pixels, of the measurement data relative to the full sensor
    virtual int origin_x() const = 0;

    /// @brief Y location, in pixels, of the measurement data relative to the full sensor
    virtual int origin_y() const = 0;

    /// @brief Measurement type
    virtual DataType type() const = 0;

    /// @brief Is the measurement pixel data in big endian or little endian format
    virtual bool is_big_endian() const = 0;

    /// @brief Is the pixel data flipped horizontally around the center
    virtual bool is_flipped_horizontally() const = 0;

    /// @brief Is the pixel data flipped vertically around the center
    virtual bool is_flipped_vertically() const = 0;

    /// @brief Is the raw pixel data sorted top-to-bottom (instead of native order of sensor)
    virtual bool is_raw_data_sorted() const = 0;

    /// @brief Size of each pixel of data in the measurement data
    virtual int pixel_size() const = 0;

    /// @brief Get the sensor (ake imaging chip) temperature data (in deg C) if present.
    ///  There are 4 corner or quadrant temperature sensors on the chip which can be
    ///  read after an ambient data capture. They are reported in the following order
    ///  Top Left, Top Right, Bottom Left, Bottum Right. 
    ///  If the temperature data is not included with this measurement then retuned std::optional
    ///  will be empty.
    virtual std::optional<std::array<float,TofComm::KLV_NUM_TEMPERATURES>> sensor_temperatures() const = 0;

    /// @brief Get the integration time setting that was active when the measurement was collected.
    ///  The sensor time in micro-seconds is used during the DCS distance collection phase,.
    ///  If the integration time data is not included with this measurement then std::nullopt is returned
    virtual std::optional<uint16_t> integration_time() const = 0;

    /// @brief Get illuminator information that was recorded at the time of the measurement.
    ///
    /// The following information is recorded just after the measurement is acquired:
    /// - Which LED segments were enabled
    /// - Temperature
    /// - VLED voltage applied to the LEDs
    /// - photodiode reading for photodiode near the LEDs. 
    /// If the illuminator information data was not included with this measurement then std::nullopt is returned
    virtual std::optional<TofComm::illuminator_info_t> illuminator_info() const = 0;

    /// @brief Get the modulation frequency setting (in HZ) that was active when the measurement was collected.
    ///  If no data is found in the measurement header then std::nullopt is returned
    virtual std::optional<uint32_t> modulation_frequency() const = 0;

    /// @brief Get the horizontal binning setting that was active when the measurement was collected.
    ///  Binning refers here to the sensor setting which combines rectangular
    ///  neighborhoods of pixels into larger "super-pixels." It reduces the
    ///  resolution of the output image to (width / binning_h) x (height / binning_v).
    ///  The default values binning_h = binning_v = 0 is considered the same
    ///  as binning_h = binning_v = 1 (no subsampling).
    ///  If no data is found in the measurement header then std::nullopt is returned
    virtual std::optional<uint8_t> horizontal_binning() const = 0;

    /// @brief Get the vertical binning setting that was active when the measurement was collected.
    ///  Binning refers here to the sensor setting which combines rectangular
    ///  neighborhoods of pixels into larger "super-pixels." It reduces the
    ///  resolution of the output image to (width / binning_h) x (height / binning_v).
    ///  The default values binning_h = binning_v = 0 is considered the same
    ///  as binning_h = binning_v = 1 (no subsampling).
    ///  If no data is found in the measurement header then std::nullopt is returned
    virtual std::optional<uint8_t> vertical_binning() const = 0;

    /// @brief Get the DLL settings that were active when the measurement was collected.
    ///  DLL settings include: 
    ///    - enabled
    ///    - coarse step
    ///    - fine step
    ///    - finest step
    ///  If no data is found in the measurement header then std::nullopt is returned
    virtual std::optional<std::array<uint8_t,TofComm::KLV_NUM_DLL_BYTES>> dll_settings() const = 0;

    /// @brief Get Vector Sequence Mode (VSM) information that was recorded at the time of the measurement.
    ///
    /// If the VSM information data was not included with this measurement then std::nullopt is returned
    virtual std::optional<TofComm::VsmControl_T> vsm_info() const = 0;

    /// @brief Get the frame timestamp that was recorded at the time of the measurement.
    /// The timestamp is recorded from the system millisecond timer.
    ///
    /// If the timestamp was not included with this measurement then std::nullopt is returned
    virtual std::optional<uint32_t> frame_timestamp() const = 0;

    /// @brief Get the frame crcs that were calculated by the sensor prior to streaming the data.
    ///
    /// If the CRCs were not included with this measurement then std::nullopt is returned
    virtual std::optional<std::vector<uint32_t>> frame_crcs() const = 0;

    /// @brief Determine whether CRC errors were detected.
    ///
    /// @return true if CRCs were in the frame KLV data and failed to match what was calculated.
    virtual bool crc_errors() const = 0;
};


/// @brief Factory function to create new Measurement_T derived object from a std vector of bytes
/// @param buffer vector of bytes containing raw recieved measurement data
/// @return Shared pointer to concrete measurement instance
std::shared_ptr<tofcore::Measurement_T> create_measurement(const std::vector<std::byte>& buffer,
                                                           log_callback_t log_callback = nullptr);

} //end namespace tofcore


#endif // __TOFCORE_MEASUREMENT_T_H__
