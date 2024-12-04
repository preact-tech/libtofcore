/**
 * Copyright (C) 2020 Espros Photonics Corporation
 *
 *
 * @defgroup iterator_index_in_lut Iterator for input index with lookUp table
 * @ingroup iterator
 * @brief Calculates the indexes for the input data for the calculations
 *
 * @{
 */
#ifndef ITERATOR_INDEX_IN_LUT_H_
#define ITERATOR_INDEX_IN_LUT_H_

#include "CommandTypes.hpp"
#include "IteratorIndexIn.h"

//! Iterator for input index with lookUp table
/*!
 * Calculates the indexes for the input data for the calculations
 */
class IteratorIndexInLut: public IteratorIndexIn
{
public:
    IteratorIndexInLut();
    void init(const uint32_t width, const uint32_t height,
              bool horizontalFlip = false, bool verticalFlip = false);
    uint32_t getNext();

protected:
    uint32_t m_currentIndex;               ///<Current Index for the LookUp table
    uint32_t m_indexLut[TofComm::IMAGER_WIDTH * TofComm::IMAGER_HEIGHT];   ///<LookUp table for the indexes
};

#endif /* ITERATOR_INDEX_IN_LUT_H_ */

/** @} */
