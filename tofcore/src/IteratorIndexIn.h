/**
 * Copyright (C) 2020 Espros Photonics Corporation
 *
 *
 * @defgroup iterator_index_in Iterator for input index
 * @ingroup iterator
 * @brief Calculates the indexes for the input data for the calculations
 *
 * @{
 */
#ifndef ITERATOR_INDEX_IN_H_
#define ITERATOR_INDEX_IN_H_

#include <cstdint>

//! Iterator for input index
/*!
 * Calculates the indexes for the input data for the calculations
 */
class IteratorIndexIn
{
public:
    IteratorIndexIn();
    void init(const uint32_t width, const uint32_t height,
              bool horizontalFlip = false, bool verticalFlip = false);
    uint32_t getNext();
    uint32_t getIndex(const uint32_t outputIndex);

protected:
    uint32_t calcNextIndex();

    uint32_t m_indexOut;                   ///<Current index for the output
    int32_t m_addPerLineBreak;             ///<Value to add after end of line
    uint32_t m_halfHeight;                 ///<Helper variable to optimize speed
    uint32_t m_width;                      ///<Image width in pixels
    uint32_t m_x;                          ///<X coordinate
    uint32_t m_y;                          ///<Y coordinate
    bool     m_horizontalFlip;
    bool     m_verticalFlip;
};

#endif /* ITERATOR_INDEX_IN_H_ */

/** @} */
