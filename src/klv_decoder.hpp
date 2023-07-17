#if !defined(_TOFCORE_KLV_DECODER_HPP_)
#define _TOFCORE_KLV_DECODER_HPP_

#include "MetaDataTypes.hpp"
#include "TofEndian.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <optional>
#include <utility>
#include <vector>

namespace tofcore
{

/// @brief Utility class to quickly search a buffer of data for a specific KLV entity
class KLVDecoder
{
public:
    using key_type = std::uint16_t;
    using length_type = std::uint16_t;
    using buf_ptrtype = std::byte*;
    using size_type = std::size_t;

    KLVDecoder(buf_ptrtype begin, const buf_ptrtype end);

    std::pair<buf_ptrtype, buf_ptrtype> find(key_type needle) const;

protected:
    const buf_ptrtype m_begin;
    const buf_ptrtype m_end;
};

/// @brief Search the provided KLV data for the binning settings and return the vertical and horizontal values as bytes.
/// @param klv 
/// @return std::nullopt if the data is not found.
/// @return std::option<[horizontal, vertical]> when the data is found
std::optional<std::array<uint8_t, 2>> decode_binning(const KLVDecoder& klv);

/// @brief Search the provided KLV data for the DLL configuration
/// @param klv 
/// @return std::nullopt if the data is not found.
/// @return std::option<[enabled, coarse, fine, finest]> when the data is found
std::optional<std::array<uint8_t, TofComm::KLV_NUM_DLL_BYTES>> decode_dll_settings(const KLVDecoder& klv);

/// @brief Search the provided KLV data for the integration time settings and return the list of values.
/// @param klv 
/// @return std::nullopt if the data is not found.
/// @return std::option<[int0,int1,int2]> when the data is found
std::optional<std::array<uint16_t, TofComm::KLV_NUM_INTEGRATION_TIMES>> decode_integration_times(const KLVDecoder& klv);

/// @brief Search the provided KLV data for the illuminator info settings and return provide values.
/// @param klv 
/// @return std::nullopt if the data is not found.
/// @return std::option<illuminator_info_t> when the data is found
std::optional<TofComm::illuminator_info_t> decode_illuminator_info(const KLVDecoder& klv);

/// @brief Search the provided KLV data for the modulation frequency setting.
/// @param klv 
/// @return std::nullopt if the data is not found.
/// @return std::option<uint32_t> when the data is found
std::optional<std::vector<uint32_t>> decode_modulation_frequencies(const KLVDecoder& klv);

/// @brief Search the provided KLV data for the chip temperature and return
///        the temperature data in degrees C if found.
/// @param klv 
/// @return std::nullopt if the data is not found.
/// @return std::option<[UL,UR,LL,LR]> when the data is found
std::optional<std::array<float, TofComm::KLV_NUM_TEMPERATURES>> decode_sensor_temperatures(const KLVDecoder& klv);

} //end namespace tofcore
#endif //_TOFCORE_KLV_DECODER_HPP_
