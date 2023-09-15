#include "klv_decoder.hpp"
#include "TofCommand_IF.hpp"

namespace tofcore
{

KLVDecoder::KLVDecoder(buf_ptrtype begin, const buf_ptrtype end) 
    : m_begin(begin), m_end(end) 
{
}

std::pair<KLVDecoder::buf_ptrtype, KLVDecoder::buf_ptrtype> KLVDecoder::find(key_type needle) const
{
    constexpr auto MIN_ELEMENT_SIZE = sizeof(key_type) + sizeof(length_type);
    auto cur = m_begin;
    while((size_t)std::distance(cur, m_end) > MIN_ELEMENT_SIZE)
    {
        key_type key {0};
        TofComm::BE_Get(key, cur);
        //key's should never be 0, treate this condition as a stopping point
        if(0 == key) 
        {
            break;
        }
        std::advance(cur, sizeof(key_type));
        length_type len {0};
        TofComm::BE_Get(len, cur);
        std::advance(cur, sizeof(length_type));
        if((key == needle) && (std::distance(cur, m_end) >= len)) 
        {
            //we found a key that looks valid.
            return {cur, cur+len};
        }
        if((length_type)std::distance(cur, m_end) >= len)
        {
            std::advance(cur, len);
        }
    }
    return {m_end, m_end};
}


std::optional<std::array<uint8_t, 2>> decode_binning(const KLVDecoder& klv)
{
    auto data = klv.find(TofComm::KLV_BINNING_KEY);
    if(data.first == data.second || std::distance(data.first, data.second) != 2) 
    {
        return std::nullopt;
    }
    return {{std::to_integer<uint8_t>(data.first[0]), std::to_integer<uint8_t>(data.first[1])}};
}


std::optional<std::array<uint8_t, TofComm::KLV_NUM_DLL_BYTES>> decode_dll_settings(const KLVDecoder& klv)
{
    auto data = klv.find(TofComm::KLV_DLL_SETTINGS_KEY);
    if(data.first == data.second || std::distance(data.first, data.second) != TofComm::KLV_NUM_DLL_BYTES)
    {
        return std::nullopt;
    }
    std::array<uint8_t, TofComm::KLV_NUM_DLL_BYTES> retval {};
    std::copy((uint8_t*)data.first, (uint8_t*)data.second, retval.begin());
    return retval;
}


std::optional<std::array<uint16_t,TofComm::KLV_NUM_INTEGRATION_TIMES>> decode_integration_times(const KLVDecoder& klv)
{
    auto data = klv.find(TofComm::KLV_INTEGRATION_TIMES_KEY);
    auto numTimes = std::distance(data.first, data.second) / sizeof(uint16_t);
    if(data.first == data.second || (numTimes < 1))
    {
        return std::nullopt;
    }
    // The integration times key value field consist of up to KLV_NUM_INTEGRATION_TIMES
    // uint16_t big endian values in micro-seconds [int0, int1, int2]
    std::array<uint16_t, TofComm::KLV_NUM_INTEGRATION_TIMES> values;
    numTimes = std::min(numTimes, TofComm::KLV_NUM_INTEGRATION_TIMES);
    for(size_t i = 0; i < numTimes; ++i)
    {
        auto temp = uint16_t{0};
        TofComm::BE_Get(temp, data.first + (i*2));
        values[i] = temp;
    }
    return {values};
}


std::optional<TofComm::illuminator_info_t> decode_illuminator_info(const KLVDecoder& klv)
{
    auto data = klv.find(TofComm::KLV_ILLUMINATOR_INFO_KEY);
    if(data.first == data.second || std::distance(data.first, data.second) < 9) 
    {
        return std::nullopt;
    }
    //The illumination info key value field consists of the following: 
    // - 1 byte: LED segment enable mask 
    // - 1 int32_t: LED temperature in milli degrees C, convert to floating point degree C
    // - 1 uint16_t: vled voltage in mV - convert to floating point volts
    // - 1 uint16_t: photodiode voltage in mV - convert to floating point volts
    TofComm::illuminator_info_t illum_info;
    illum_info.led_segments_enabled = static_cast<uint8_t>(*data.first);
    int32_t i32 {0};
    TofComm::BE_Get(i32, data.first + 1);
    illum_info.temperature_c = i32 / 1000.0;
    uint16_t u16 {0};
    TofComm::BE_Get(u16, data.first + 5);
    illum_info.vled_v = u16 / 1000.0;
    TofComm::BE_Get(u16, data.first + 7);
    illum_info.photodiode_v = u16 / 1000.0;
    return {illum_info};
}


std::optional<std::vector<uint32_t>> decode_modulation_frequencies(const KLVDecoder& klv)
{
    auto data = klv.find(TofComm::KLV_MODULATION_FREQUENCY_KEY);
    if(data.first == data.second || std::distance(data.first, data.second) < 4) 
    {
        return std::nullopt;
    }
    //The modulation frequency value field consist of: 
    // - A list of at least 1 uint32_t value representing the mod frequency in HZ. 
    std::vector<uint32_t> values;
    for(auto i = data.first; i != data.second; i += sizeof(uint32_t))
    {
        uint32_t temp;
        TofComm::BE_Get(temp, i);
        values.push_back(temp);
    }
    return {values};
}


std::optional<std::array<float, TofComm::KLV_NUM_TEMPERATURES>> decode_sensor_temperatures(const KLVDecoder& klv)
{
    auto data = klv.find(TofComm::KLV_SENSOR_TEMPERATURE_KEY);

    if(data.first == data.second || std::distance(data.first, data.second) != (3 * TofComm::KLV_NUM_TEMPERATURES))
    {
        return std::nullopt;
    }
    //Sensor temperature data consists of:
    // - 4 int16_t big endian temperature values.
    // - 4 uin8_t temperature calibration values.
    int16_t* raw_values {(int16_t*)data.first};
    auto cal_values {data.first + (sizeof(int16_t)*4)};
    std::array<float, TofComm::KLV_NUM_TEMPERATURES> temp_degC;
    for(std::size_t i = 0; i != temp_degC.size(); ++i) 
    {
        int16_t raw_value {0};
        auto calibration_val = std::to_integer<uint8_t>(cal_values[i]);
        TofComm::BE_Get(raw_value, (uint8_t*)(raw_values + i));
        float normalizedTempDegC = calibration_val/4.7f - 299;
        temp_degC[i] = (raw_value - 0x2000) * 0.134 + normalizedTempDegC;
    }
    return {temp_degC};
}

}
