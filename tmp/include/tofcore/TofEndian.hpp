/**
 * @file TofEndian.hpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Support for Endian conversions
 * @{
 */
#ifndef TOFENDIAN_HPP
#define TOFENDIAN_HPP

#include <boost/endian/conversion.hpp>

namespace TofComm
{

/**
 * This template function supports converting fixed-sized numeric types from big
 * endian (using Boost) even when they originate in memory not aligned for that
 * type (something the Boost library doesn't take care of for us).
 * @param dst Reference to the location to which the converted value is written.
 * @param src The non-NULL pointer to the location from which the data is read.
 */
template <class NumericType> inline void BE_Get(NumericType& dst, const void* src)
{
    if (src != nullptr)
    {
        NumericType tmp;
        memmove(&tmp, src, sizeof(tmp));
        dst = boost::endian::big_to_native(tmp);
    }
}
/**
 * This template function supports converting fixed-sized numeric types to big
 * endian (using Boost) and storing the result at a specified address. That
 * destination need not be naturally aligned for the numeric type - something
 * the Boost library doesn't take care of for us.
 * @param dst Non-NULL pointer to location to store the big endian data
 * @param src The value to be converted to big endian format.
 */
template <class NumericType> inline void BE_Put(void* dst, const NumericType src)
{
    if (dst != nullptr)
    {
        const NumericType tmp { boost::endian::native_to_big(src) };
        memmove(dst, &tmp, sizeof(tmp));
    }
}

/**
 * This template function supports converting fixed-sized numeric types from little
 * endian (using Boost) even when they originate in memory not aligned for that
 * type (something the Boost library doesn't take care of for us).
 * @param dst Reference to the location to which the converted value is written.
 * @param src The non-NULL pointer to the location from which the data is read.
 */
template <class NumericType> inline void LE_Get(NumericType& dst, const void* src)
{
    if (src != nullptr)
    {
        NumericType tmp;
        memmove(&tmp, src, sizeof(tmp));
        dst = boost::endian::little_to_native(tmp);
    }
}
/**
 * This template function supports converting fixed-sized numeric types to little
 * endian (using Boost) and storing the result at a specified address. That
 * destination need not be naturally aligned for the numeric type - something
 * the Boost library doesn't take care of for us.
 * @param dst Non-NULL pointer to location to store the little endian data
 * @param src The value to be converted to little endian format.
 */
template <class NumericType> inline void LE_Put(void* dst, const NumericType src)
{
    if (dst != nullptr)
    {
        const NumericType tmp { boost::endian::native_to_little(src) };
        memmove(dst, &tmp, sizeof(tmp));
    }
}

/**
 * This template function supports appending fixed-sized numeric types to a buffer 
 * @param dst Non-NULL pointer to location to store the big-endian data
 * @param src The value to be converted to big endian format.
 * @return New pointer pointing one byte past the freshly inserted value
 */
template<typename T> inline std::byte* BE_Append(std::byte* dst, const T& src)
{
    BE_Put(dst, src);
    return dst + sizeof(src);
}


} // namespace TofComm

#endif // TOFENDIAN_HPP

/** @} */
