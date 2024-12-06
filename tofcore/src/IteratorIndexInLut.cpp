/**
 * Copyright (C) 2020 Espros Photonics Corporation
 *
 *
 * @addtogroup iterator_index_in_lut
 *
 * @{
 */
#include "IteratorIndexInLut.h"

/**
 * @brief Constructor
 */
IteratorIndexInLut::IteratorIndexInLut()
{
    m_currentIndex = 0;
}

/**
 * @brief init
 *
 * Call this function before running through an image.
 *
 * @param width Width of the image in pixels
 * @param height Height of the image in pixels
 */
void IteratorIndexInLut::init(const uint32_t width, const uint32_t height,
                              bool horizontalFlip, bool verticalFlip)
{
    bool changed = ( (m_width != width) ||
                     (m_halfHeight != (height / 2)) ||
                     (verticalFlip != m_verticalFlip) ||
                     (horizontalFlip != m_horizontalFlip));

    IteratorIndexIn::init(width, height, horizontalFlip, verticalFlip);

    if (changed)
    {
        uint32_t numPixel = width * height;
        for (uint32_t i = 0; i < numPixel; i++)
        {
            m_indexLut[i] = IteratorIndexIn::getNext();
        }
    }

    m_currentIndex = 0;
}

/**
 * @brief Get next index
 *
 * Call this function for each pixel to get the sorted index.
 * Important: Before using this function call the "init" function.
 * Here only the Lookup Table is accessed.
 *
 * @return Sorted index to use for the input data
 */
uint32_t IteratorIndexInLut::getNext()
{
    return m_indexLut[m_currentIndex++];
}

/** @} */
