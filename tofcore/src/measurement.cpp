
#include "CommandTypes.hpp"
#include "klv_decoder.hpp"
#include "Measurement_T.hpp"
#include "TofEndian.hpp"
#include "TofCommand_IF.hpp"
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
    Measurement(const std::vector<byte_t>& buffer)
    {
        BufferView<byte_t> view {buffer.data(), buffer.size()};
        this->init(view);
    }

    Measurement() = default;
    Measurement(const Measurement&) = default;
    virtual ~Measurement() = default;

    /// @brief Width, in pixels, for measurement data
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

    /// @brief Get a raw view into the measurement buffer
    virtual BufferView<byte_t> pixel_buffer() const override
    {
        return {m_pixel_data.data(), m_pixel_data.size()};
    }

    /// @brief Obtain a view of one of the dcs frames from a DCS mesaurement
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

    /// @brief Obtain a view of the grayscale (ambient) data from a grayscale measurement
    /// @return view of grayscale data
    /// @return empty view if not a grayscale measurement
    virtual BufferView<int16_t> ambient() const override
    {
        //TODO check DataType
        std::size_t size = width() * height();
        auto ptr = reinterpret_cast<int16_t*>(m_pixel_data.data());
        return {ptr, size};
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

    /// @brief 
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

protected:

    /// @brief Initialize the Measurement with a buffer of raw measurement data (header and all)
    void init(const BufferView<byte_t>& input)
    {
        auto begin = input.begin();
        auto end = input.end();
        //The header is always at the beginning of the data, the header version tells us about the data in the header, it's size, etc. 
        auto version = *(begin + TofComm::V0_T1_VERSION_INDEX);
        switch(std::to_integer<uint8_t>(version))
        {
            case 2:
            case 3:
            {
                uint16_t data_offset = 0;
                int16_t type = 0;
                TofComm::BE_Get(data_offset, begin + TofComm::V0_T1_DATA_OFFSET_INDEX);
                TofComm::BE_Get(m_width, begin + TofComm::V0_T1_WIDTH_INDEX);
                TofComm::BE_Get(m_height, begin + TofComm::V0_T1_HEIGHT_INDEX);
                TofComm::BE_Get(type, begin + TofComm::V0_T1_TYPE_INDEX);
                TofComm::BE_Get(m_origin_x, begin + TofComm::V0_T1_ORIGIN_X_INDEX);
                TofComm::BE_Get(m_origin_y, begin + TofComm::V0_T1_ORIGIN_Y_INDEX);
                m_type = static_cast<DataType>(type);
                //Copy the remaining user data (aka KLV) over to m_meta_data
                m_meta_data.resize(data_offset - TofComm::V0_T1_USER_DATA_INDEX);
                memset(m_meta_data.data(), 0, m_meta_data.size());

                std::copy(begin + TofComm::V0_T1_USER_DATA_INDEX, 
                          begin + data_offset,
                          m_meta_data.begin());

                //Copy the pixel data over, do not reorder each multibyte pixel at this time.
                auto distance = std::distance(begin + data_offset, end);
                m_pixel_data.resize(distance);
                memset(m_pixel_data.data(), 0, m_pixel_data.size());
                if(DataType::DISTANCE_AMPLITUDE == m_type )
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
            default:
            {
                //NOT SURE WHAT TO DO FOR UNKNOWN HEADER VERSIONS?
                break;
            }
        }
    }

protected:
    uint16_t m_width {0};
    uint16_t m_height {0};
    uint16_t m_origin_x {0};
    uint16_t m_origin_y {0};
    DataType m_type {DataType::UNKNOWN};
    bool m_big_endian {true};
    SharedBuffer<byte_t> m_meta_data;
    SharedBuffer<byte_t> m_pixel_data;
};


std::shared_ptr<tofcore::Measurement_T> create_measurement(const std::vector<std::byte>& buffer)
{
    return std::make_shared<tofcore::Measurement>(buffer);
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
