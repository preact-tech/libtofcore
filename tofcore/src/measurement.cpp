
#include "CommandTypes.hpp"
#include "crc32.h"
#include "IteratorIndexInLut.h"
#include "klv_decoder.hpp"
#include "Measurement_T.hpp"
#include "TofEndian.hpp"
#include "TofCommand_IF.hpp"
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

namespace tofcore
{

/// @brief Simple buffer class to help manage shared ownership of a chuck of data.
template<typename T>
struct SharedBuffer : public std::tuple<std::shared_ptr<T[]>, std::size_t>
{
    SharedBuffer(std::shared_ptr<T[]>& d, std::size_t l) :
        std::tuple<std::shared_ptr<T[]>, std::size_t>(d, l)
    {}

    SharedBuffer(std::size_t l) :
        std::tuple<std::shared_ptr<T[]>, std::size_t>(std::make_shared<T[]>(l), l)
    {}

    SharedBuffer() = default;

    void resize(std::size_t l)
    {
        std::get<std::shared_ptr<T[]>>(*this).reset(new T[l]);
        std::get<std::size_t>(*this) = l;
    }

    auto data()
    {
        return std::get<std::shared_ptr<T[]>>(*this).get();
    }

    auto data() const
    {
        return std::get<std::shared_ptr<T[]>>(*this).get();
    }

    auto size() const
    {
        return std::get<std::size_t>(*this);
    }

    auto begin() const
    {
        return data();
    }

    auto end() const
    {
        return data()+size();
    }
};

/// @brief Concrete implmentation of the Measurement_T interface
class Measurement : public Measurement_T
{
public:

    /// @brief Construct a Measurement from a vector of bytes. 
    ///        The Measurement will validate and make a copy of the measurement data
    ///        that it needs. 
    Measurement(const std::vector<byte_t>& buffer,
                log_callback_t log_callback)
    {
        BufferView<byte_t> view {buffer.data(), buffer.size()};
        this->init(view, log_callback);
    }

    Measurement() = default;
    Measurement(const Measurement&) = default;
    virtual ~Measurement() = default;

    void check_crcs(const uint16_t data_offset,
                    const BufferView<byte_t>& input,
                    log_callback_t log_callback)
    {
        auto&& frameCrcs = frame_crcs();
        if (frameCrcs)
        {
            const uint32_t numCrcs = frameCrcs->size();
            const uint8_t* firstPixel = reinterpret_cast<const uint8_t*>(input.begin()) + data_offset;
            const auto frameSize = (reinterpret_cast<const uint8_t*>(input.end()) - firstPixel) / numCrcs;
            for (uint32_t i = 0; i < numCrcs; ++i)
            {
                const uint32_t validCrc = frameCrcs->data()[i];
                const uint32_t calcCrc = calcCrc32((firstPixel + i * frameSize), frameSize);
                if (validCrc == calcCrc)
                {
                    if (log_callback)
                    {
                        std::stringstream ss {};
                        ss << "Valid CRC: 0x" << std::setw(8) << std::hex << std::setfill('0')
                           << validCrc << " of " << std::dec << frameSize << " bytes";
                        log_callback(ss.str(), LOG_LVL_DBG_MID);
                    }
                }
                else
                {
                    m_crcErrors = true;
                    if (log_callback)
                    {
                        std::stringstream ss {};
                        ss << "INVALID CRC of " << frameSize << " bytes: 0x" << std::setw(8)
                           << std::hex << std::setfill('0') << calcCrc << " (0x" << validCrc << ")";
                        log_callback(ss.str(), LOG_LVL_ERROR);
                    }
                }
            }
        }

    }

    std::string describe_frame()
    {
        const char *frameType { "UNKNOWN" };
        bool isRaw { false };
        switch (m_type)
        {
            case DataType::AMBIENT:
                frameType = "AMBIENT";
                isRaw = !m_is_sorted;
                break;
            case DataType::AMPLITUDE:
                frameType = "AMPLITUDE";
                break;
            case DataType::DCS_DIFF_AMBIENT:
                frameType = "DCS_DIFF_AMBIENT";
                isRaw = !m_is_sorted;
                break;
            case DataType::DCS:
                frameType = "DCS";
                isRaw = !m_is_sorted;
                break;
            case DataType::DISTANCE_AMPLITUDE:
                frameType = "DISTANCE_AMPLITUDE";
                break;
            case DataType::DISTANCE:
                frameType = "DISTANCE";
                break;
            case DataType::GRAYSCALE:
                frameType = "GRAYSCALE";
                break;
            default:
            case DataType::UNKNOWN:
                break;
        }
        std::stringstream ss {};
        auto&& timeStamp = frame_timestamp();
        if (timeStamp)
        {
            ss << "[" << *timeStamp << "] ";
        }
        ss << std::to_string(m_width) << " X " << std::to_string(m_height) << " " << frameType << " frame.";
        ss << " HFlip: " << (m_flipped_horizontally ? "Y" : "N") << "; VFlip: " << (m_flipped_vertically ? "Y" : "N");
        ss << "; Raw: " << (isRaw ? "Y" : "N");
        auto&& intTime = integration_time();
        if (intTime)
        {
            ss << "; intTime: " << *intTime;
        }
        auto&& modFreq = modulation_frequency();
        if (modFreq)
        {
            ss << "; modFreq: " << *modFreq;
        }
        auto&& ibData = illuminator_info();
        if (ibData)
        {
            ss << "; ibTemp: " << std::setprecision(4) << ibData->temperature_c;
        }
        auto&& sensorTemps = sensor_temperatures();
        if (sensorTemps)
        {
            ss << "; sensorTemps: [";
            const char* separator = "";
            for (const auto& sensorTemp : *sensorTemps)
            {
                ss << separator << std::setprecision(4) << sensorTemp;
                separator = ", ";
            }
            ss << "]";
        }
        auto&& frameCrcs = frame_crcs();
        if (frameCrcs)
        {
            ss << "; crcs: [";
            const char* separator = "";
            for (const auto& crc : *frameCrcs)
            {
                ss << separator << "0x" << std::setw(8) << std::hex << std::setfill('0') << crc;
                separator = ", ";
            }
            ss << "]";
        }
        return ss.str();
    }

    virtual int width() const override
    {
        return m_width;
    }

    /// @brief Height, in pixels, for measurement data
    virtual int height() const override
    {
        return m_height;
    }

    /// @brief Measurement data type (DCS, DISTANCE, etc)
    virtual DataType type() const override
    {
        return m_type;
    }

    virtual int origin_x() const override
    {
        return m_origin_x;
    }

    virtual int origin_y() const override
    {
        return m_origin_y;
    }

    virtual int pixel_size() const override
    {
        return 2;
    }

    virtual bool is_big_endian() const override
    {
        return m_big_endian;
    }

    virtual bool is_flipped_horizontally() const override
    {
        return m_flipped_horizontally;
    }

    virtual bool is_flipped_vertically() const override
    {
        return m_flipped_vertically;
    }

    virtual bool is_raw_data_sorted() const override
    {
        return m_is_sorted;
    }

    virtual BufferView<byte_t> pixel_buffer() const override
    {
        return {m_pixel_data.data(), m_pixel_data.size()};
    }

    /// @brief Obtain a view of one of the dcs frames from a DCS measurement
    /// @param dcs_index index of DCS frame to return 0-3
    /// @return view of requested DCS frame data
    /// @return empty view if index out of range or not a DCS measurement
    virtual BufferView<int16_t> dcs(int dcs_index) const override
    {
        if(DataType::DCS != type() || dcs_index < 0 || dcs_index > 3)
        {
            return {nullptr, 0};
        }
        std::size_t size = width() * height();
        auto ptr = reinterpret_cast<int16_t*>(m_pixel_data.data()) + size * dcs_index;
        return {ptr, size};
    }

    /// @brief Obtain a view of one of either the (DCS2-DCS0)
    ///        or (DCS3-DCS1) frames from a DCS Diff + Ambient measurement
    /// @param index 0 for (DCS2-DCS0), 1 for (DCS3-DCS1)
    /// @return view of requested frame data
    /// @return empty view if index out of range or not a DCS Diff + Ambient measurement
    virtual BufferView<int16_t> dcs_diff(int index) const override
    {
        if(DataType::DCS_DIFF_AMBIENT != type() || index < 0 || index > 1)
        {
            return {nullptr, 0};
        }
        std::size_t size = width() * height();
        auto ptr = reinterpret_cast<int16_t*>(m_pixel_data.data()) + size * index;
        return {ptr, size};
    }

    /// @brief Obtain a view of the distance data from a distance measurement
    /// @return view of distance data
    /// @return empty view if not a distance measurement
    virtual BufferView<uint16_t> distance() const override
    {
        switch(type())
        {
            case DataType::DISTANCE_AMPLITUDE:
            case DataType::DISTANCE:
            {
                std::size_t size = width() * height();
                auto ptr = reinterpret_cast<uint16_t*>(m_pixel_data.data());
                return {ptr, size};
            }
            default:
                return {nullptr, 0};
        }
    }

    /// @brief Obtain a view of the amplitude data from a amplitude measurement
    /// @return view of amplitude data
    /// @return empty view if not a amplitude measurement
    virtual BufferView<uint16_t> amplitude() const override
    {
        switch(type())
        {
            case DataType::AMPLITUDE:
            {
                //Note: I don't know if you can actually stream only amplitude, but
                // type enum exists so I'm covering the case
                std::size_t size = width() * height();
                auto ptr = reinterpret_cast<uint16_t*>(m_pixel_data.data());
                return {ptr, size};
            }
            case DataType::DISTANCE_AMPLITUDE:
            {
                std::size_t size = width() * height();
                auto ptr = reinterpret_cast<uint16_t*>(m_pixel_data.data());
                // the amplitude frame is immediately after the distance frame
                ptr += size;
                return {ptr, size};
            }
            default:
                return {nullptr, 0};
        }
    }

    /// @brief Obtain a view of the grayscale (ambient) data from a grayscale
    ///        measurement or from a DCS Diff + Ambient measurement
    /// @return view of grayscale data
    /// @return empty view if not a grayscale measurement
    virtual BufferView<int16_t> ambient() const override
    {
        switch(type())
        {
            case DataType::AMBIENT:
            case DataType::GRAYSCALE:
            {
                //Note: I don't know if you can actually stream only amplitude, but
                // type enum exists so I'm covering the case
                std::size_t size = width() * height();
                auto ptr = reinterpret_cast<int16_t*>(m_pixel_data.data());
                return {ptr, size};
            }
            case DataType::DCS_DIFF_AMBIENT:
            {
                std::size_t size = width() * height();
                auto ptr = reinterpret_cast<int16_t*>(m_pixel_data.data());
                // the ambient frame is immediately after the 2 DCS difference frames
                ptr += 2*size;
                return {ptr, size};
            }
            default:
                return {nullptr, 0};
        }
    }

    /// @brief Obtain a view of the raw bytes containing the meta_data associated with the measurement
    /// @return BufferView of meta data
    virtual BufferView<byte_t> meta_data() const override
    {
        return {m_meta_data.data(), m_meta_data.size()};
    }

    /// @brief Obtain the sensor (aka imaging chip) temperature information
    virtual std::optional<std::array<float,TofComm::KLV_NUM_TEMPERATURES>> sensor_temperatures() const override
    {
        KLVDecoder decoder {m_meta_data.begin(), m_meta_data.end()};
        return decode_sensor_temperatures(decoder);
    }

    /// @brief Obtain the integration time setting that was active at the time measurement was acquired.
    virtual std::optional<uint16_t> integration_time() const override
    {
        KLVDecoder decoder {m_meta_data.begin(), m_meta_data.end()};
        return decode_integration_time(decoder);
    }

    /// @brief Obtain the modulation frequency setting that was active at the time the measurement was collected
    virtual std::optional<uint32_t> modulation_frequency() const override
    {
        KLVDecoder decoder {m_meta_data.begin(), m_meta_data.end()};
        return decode_modulation_frequency(decoder);
    }

    /// @brief Obtain the horizontal binning setting that was active at the time the measurement was collected
    virtual std::optional<uint8_t> horizontal_binning() const override
    {
        KLVDecoder decoder {m_meta_data.begin(), m_meta_data.end()};
        auto binning = decode_binning(decoder);
        if( !binning )
        {
            return std::nullopt;
        }
        else 
        {
            return {(*binning)[0]};
        }
    }

    /// @brief Obtain the vertical binning setting that was active at the time the measurement was collected
    virtual std::optional<uint8_t> vertical_binning() const override
    {
        KLVDecoder decoder {m_meta_data.begin(), m_meta_data.end()};
        auto binning = decode_binning(decoder);
        if( !binning )
        {
            return std::nullopt;
        }
        else 
        {
            return {(*binning)[1]};
        }
    }

    /// @brief Get the DLL settings that were active when the measurement was collected.
    ///  DLL settings include: [enabled, coarse step, fine step, finest step]
    virtual std::optional<std::array<uint8_t, TofComm::KLV_NUM_DLL_BYTES>> dll_settings() const override
    {
        KLVDecoder decoder {m_meta_data.begin(), m_meta_data.end()};
        return decode_dll_settings(decoder);
    }

    virtual std::optional<TofComm::illuminator_info_t> illuminator_info() const override
    {
        KLVDecoder decoder {m_meta_data.begin(), m_meta_data.end()};
        return decode_illuminator_info(decoder);
    }

    virtual std::optional<TofComm::VsmControl_T> vsm_info() const override
    {
        KLVDecoder decoder {m_meta_data.begin(), m_meta_data.end()};
        return decode_vsm_info(decoder);
    }

    virtual std::optional<uint32_t> frame_timestamp() const override
    {
        KLVDecoder decoder {m_meta_data.begin(), m_meta_data.end()};
        return decode_frame_timestamp(decoder);
    }

    virtual std::optional<std::vector<uint32_t>> frame_crcs() const override
    {
        KLVDecoder decoder {m_meta_data.begin(), m_meta_data.end()};
        return decode_frame_crcs(decoder);
    }

    virtual bool crc_errors() const override
    {
        return m_crcErrors;
    }


protected:

    /// @brief Initialize the Measurement with a buffer of raw measurement data (header and all)
    void init(const BufferView<byte_t>& input,
              log_callback_t log_callback)
    {
        auto begin = input.begin();
        auto end = input.end();
        //The header is always at the beginning of the data, the header version tells us about the data in the header, it's size, etc. 
        auto version = *(begin + TofComm::HEADER_VERSION_INDEX);
        const uint8_t versionValue { std::to_integer<uint8_t>(version) };
        switch(versionValue)
        {
            case 0:
            case 1:
            {
                // Espros headers not supported
                break;
            }
            default: // Assumes backward compatibility as version increases
            {
                uint16_t data_offset = 0;
                int16_t type = 0;
                TofComm::BE_Get(data_offset, begin + TofComm::HEADER_DATA_OFFSET_INDEX);
                TofComm::BE_Get(m_width, begin + TofComm::HEADER_WIDTH_INDEX);
                TofComm::BE_Get(m_height, begin + TofComm::HEADER_HEIGHT_INDEX);
                TofComm::BE_Get(type, begin + TofComm::HEADER_TYPE_INDEX);
                TofComm::BE_Get(m_origin_x, begin + TofComm::HEADER_ORIGIN_X_INDEX);
                TofComm::BE_Get(m_origin_y, begin + TofComm::HEADER_ORIGIN_Y_INDEX);

                if (versionValue > 3) // Added flags as of version 4
                {
                    const uint8_t flags = std::to_integer<uint8_t>(*(begin + TofComm::HEADER_DATA_FLAGS_INDEX));
                    m_flipped_horizontally = ((flags & TofComm::HORIZONTAL_FLIP) != 0);
                    m_flipped_vertically = ((flags & TofComm::VERTICAL_FLIP) != 0);
                    m_is_sorted = ((flags & TofComm::RAW_DATA_IN_NATIVE_ORDER) == 0);
                }
                else
                {
                    m_flipped_horizontally = false;
                    m_flipped_vertically = false;
                    m_is_sorted = true;
                }

                m_type = static_cast<DataType>(type);
                //Copy the remaining user data (aka KLV) over to m_meta_data
                m_meta_data.resize(data_offset - TofComm::HEADER_USER_DATA_INDEX);
                memset(m_meta_data.data(), 0, m_meta_data.size());

                std::copy(begin + TofComm::HEADER_USER_DATA_INDEX, 
                          begin + data_offset,
                          m_meta_data.begin());

                //Copy the pixel data over, do not reorder each multibyte pixel at this time.
                auto distance = std::distance(begin + data_offset, end);
                m_pixel_data.resize(distance);
                memset(m_pixel_data.data(), 0, m_pixel_data.size());

                check_crcs(data_offset, input, log_callback);

                if (log_callback)
                {
                    auto&& msg = describe_frame();
                    log_callback(msg, LOG_LVL_DBG_HI);
                }

                if (!m_is_sorted && (DataType::AMBIENT == m_type))
                {
                    process_raw_ambient(data_offset, input, 0);     // sort AMBIENT rows and scale pixels in image
                }
                else if (!m_is_sorted && (DataType::DCS == m_type))
                {
                    process_raw_dcs(data_offset, input);            // sort DCS rows and scale pixels in image
                }
                else if (!m_is_sorted && (DataType::DCS_DIFF_AMBIENT == m_type))
                {
                    process_raw_delta_dcs(data_offset, input);  // sort DELTA_DCS rows, sort/scale ambient
                }
                else if (DataType::DISTANCE_AMPLITUDE == m_type)
                {
                    //Distance Amplitude data is delivered interleaved, reorder to be planar.
                    auto distance_iter = m_pixel_data.begin();
                    auto amplitude_iter = m_pixel_data.begin() + m_width * m_height * sizeof(uint16_t);
                    for(auto i = begin + data_offset; i != end; i += 4, distance_iter += 2, amplitude_iter += 2)
                    {
                        *distance_iter = *i;
                        *(distance_iter+1) = *(i+1);
                        *(amplitude_iter) = *(i+2);
                        *(amplitude_iter+1) = *(i+3);
                    }
                }
                else
                {
                    std::copy(begin + data_offset, end, m_pixel_data.begin());
                }
                m_big_endian = true;
                break;
            }
        }
    }

    void process_raw_ambient(const uint16_t input_offset,
                             const BufferView<byte_t>& input,
                             const uint16_t output_offset)
    {
        const int16_t *pDataIn = reinterpret_cast<const int16_t*>(input.data() + input_offset);
        int16_t *pDataResult = reinterpret_cast<int16_t*>(m_pixel_data.data() + output_offset);
        /*
         * m_iterator provides the sequence of indices into the input data that
         * corresponds to the "top-to-bottom" ordering desired for the output.
         */
        m_iterator.init(m_width,
                        m_height,
                        m_flipped_horizontally,
                        m_flipped_vertically);

        const uint32_t numPixels = m_width * m_height;
        for (uint32_t i = 0; i < numPixels; i++)
        {
            uint32_t indexIn = m_iterator.getNext();
            auto currentValue = pDataIn[indexIn];
            // Mask to 12 bits (Data only)
            currentValue &= 0xFFF;
            // Convert to signed data as per EPC660 datasheet
            currentValue -= 2048;
            pDataResult[i] = currentValue;
        }
    }

    void process_raw_dcs(const uint16_t input_offset, const BufferView<byte_t>& input)
    {
        const uint32_t numPixels =  m_width * m_height;
        auto pIn_dcs0 = reinterpret_cast<const int16_t*>(input.data() + input_offset);
        auto pIn_dcs1 = pIn_dcs0 + numPixels;
        auto pIn_dcs2 = pIn_dcs1 + numPixels;
        auto pIn_dcs3 = pIn_dcs2 + numPixels;

        auto pOut_dcs0 = reinterpret_cast<int16_t*>(m_pixel_data.data());
        auto pOut_dcs1 = pOut_dcs0 + numPixels;
        auto pOut_dcs2 = pOut_dcs1 + numPixels;
        auto pOut_dcs3 = pOut_dcs2 + numPixels;
        /*
         * m_iterator provides the sequence of indices into the input data that
         * corresponds to the "top-to-bottom" ordering desired for the output.
         */
        m_iterator.init(m_width,
                        m_height,
                        m_flipped_horizontally,
                        m_flipped_vertically);

        for (uint32_t i = 0; i < numPixels; i++)
        {
            const uint32_t indexIn = m_iterator.getNext();
            /*
             * Obtain references to the input and output pixels once to
             * minimize the pointer math.
             */
            auto& out_dcs0 = *(pOut_dcs0 + i);
            auto& out_dcs1 = *(pOut_dcs1 + i);
            auto& out_dcs2 = *(pOut_dcs2 + i);
            auto& out_dcs3 = *(pOut_dcs3 + i);

            const auto& in_dcs0 = *(pIn_dcs0 + indexIn);
            const auto& in_dcs1 = *(pIn_dcs1 + indexIn);
            const auto& in_dcs2 = *(pIn_dcs2 + indexIn);
            const auto& in_dcs3 = *(pIn_dcs3 + indexIn);
            /*
             * Keep track of whether there's any saturation for the pixel.
             */
            constexpr uint16_t SATURATION_MASK = 0x1000;
            const auto sat_dcs0 = in_dcs0 & SATURATION_MASK;
            const auto sat_dcs1 = in_dcs1 & SATURATION_MASK;
            const auto sat_dcs2 = in_dcs2 & SATURATION_MASK;
            const auto sat_dcs3 = in_dcs3 & SATURATION_MASK;
            /*
             * Copy input to sorted location in output keeping only the
             * 12 bits of valid data
             */
            out_dcs0 = (in_dcs0 & 0xFFF);
            out_dcs1 = (in_dcs1 & 0xFFF);
            out_dcs2 = (in_dcs2 & 0xFFF);
            out_dcs3 = (in_dcs3 & 0xFFF);
            /*
             * Scale the data into the range of -2048 to +2047.
             */
            out_dcs0 -= 2048;
            out_dcs1 -= 2048;
            out_dcs2 -= 2048;
            out_dcs3 -= 2048;
            /*
             * Restore any saturation indicator
             */
            out_dcs0 ^= (sat_dcs0 << 2);
            out_dcs1 ^= (sat_dcs1 << 2);
            out_dcs2 ^= (sat_dcs2 << 2);
            out_dcs3 ^= (sat_dcs3 << 2);
        }
    }

    void process_raw_delta_dcs(const uint16_t input_offset, const BufferView<byte_t>& input)
    {
        /*
         * Sort the two delta DCS frames at the start of the input
         */
        auto pIn_dcs2m0 = reinterpret_cast<const int16_t*>(input.data() + input_offset);
        const uint32_t numPixels =  m_width * m_height;
        auto pIn_dcs3m1  = pIn_dcs2m0 + numPixels;
        auto pIn_ambient = pIn_dcs3m1 + numPixels;

        auto pOutDcs2m0  = reinterpret_cast<int16_t*>(m_pixel_data.data());
        auto pOutDcs3m1  = pOutDcs2m0 + numPixels;
        auto pOutAmbient = pOutDcs3m1 + numPixels;
        /*
         * m_iterator provides the sequence of indices into the input data that
         * corresponds to the "top-to-bottom" ordering desired for the output.
         */
        m_iterator.init(m_width,
                        m_height,
                        m_flipped_horizontally,
                        m_flipped_vertically);

        for (uint32_t i = 0; i < numPixels; i++)
        {
            const uint32_t indexIn = m_iterator.getNext();
            /*
             * Obtain references to the input and output pixels once to
             * minimize the pointer math.
             */
            auto& outDcs2m0  = *(pOutDcs2m0  + i);
            auto& outDcs3m1  = *(pOutDcs3m1  + i);
            auto& outAmbient = *(pOutAmbient + i);

            const auto& in_dcs2m0  = *(pIn_dcs2m0  + indexIn);
            const auto& in_dcs3m1  = *(pIn_dcs3m1  + indexIn);
            const auto& in_ambient = *(pIn_ambient + indexIn);

            outDcs2m0 = in_dcs2m0;
            outDcs3m1 = in_dcs3m1;

            int16_t amb = in_ambient;
            // Mask to 12 bits (Data only)
            amb &= 0xFFF;
            // Convert to signed data as per EPC660 datasheet
            amb -= 2048;
            outAmbient = amb;
        }
    }

protected:
    bool m_crcErrors {false};
    uint16_t m_width {0};
    uint16_t m_height {0};
    uint16_t m_origin_x {0};
    uint16_t m_origin_y {0};
    DataType m_type {DataType::UNKNOWN};
    bool m_big_endian {true};
    bool m_flipped_horizontally {false};
    bool m_flipped_vertically {false};
    bool m_is_sorted {true};
    SharedBuffer<byte_t> m_meta_data;
    SharedBuffer<byte_t> m_pixel_data;
    IteratorIndexInLut m_iterator { };
};


std::shared_ptr<tofcore::Measurement_T> create_measurement(const std::vector<std::byte>& buffer,
                                                           log_callback_t log_callback)
{
    return std::make_shared<tofcore::Measurement>(buffer, log_callback);
}

} //namespace tofcore

namespace TofComm
{

void vsmEndianConversion(VsmControl_T& vsmControl)
{
    vsmControl.m_vsmFlags = util::GetUIntFieldBigEndian(vsmControl, &VsmControl_T::m_vsmFlags);
    const uint8_t numElements { (vsmControl.m_numberOfElements <= VSM_MAX_NUMBER_OF_ELEMENTS) ?
                                vsmControl.m_numberOfElements : (uint8_t)VSM_MAX_NUMBER_OF_ELEMENTS };
    for (uint8_t n = 0; n < numElements; ++n)
    {
        VsmElement_T& element = vsmControl.m_elements[n];
        element.m_integrationTimeUs = util::GetUIntFieldBigEndian(element, &VsmElement_T::m_integrationTimeUs);
        element.m_modulationFreqKhz = util::GetUIntFieldBigEndian(element, &VsmElement_T::m_modulationFreqKhz);
    }
}

} //namespace TofComm
