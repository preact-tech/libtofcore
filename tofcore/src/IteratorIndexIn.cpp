/**
 * Copyright (C) 2020 Espros Photonics Corporation
 *
 *
 * @addtogroup iterator_index_in
 *
 * @{
 */
#include "IteratorIndexIn.h"

/**
 * @brief Constructor
 */
IteratorIndexIn::IteratorIndexIn()
{
    m_width = 0;
    m_x = 0;
    m_y = 0;
    m_indexOut = 0;
    m_addPerLineBreak = 0;
    m_halfHeight = 0;
    m_horizontalFlip = false;
    m_verticalFlip = false;
}

/**
 * @brief init
 *
 * Call this function before running through an image.
 *
 * @param imageWidth Width of the image in pixels
 * @param imageHeight Height of the image in pixels
 */
void IteratorIndexIn::init(const uint32_t imageWidth, const uint32_t imageHeight,
                           bool horizontalFlip, bool verticalFlip)
{
    m_horizontalFlip = horizontalFlip;
    m_verticalFlip = verticalFlip;
    m_width = imageWidth;
    uint32_t numPixel = imageWidth * imageHeight;
    m_halfHeight = imageHeight / 2;
    m_x = 0;
    m_y = 0;
    m_addPerLineBreak = -((2 * m_width) + m_width - 1);
    m_indexOut = numPixel - (2 * m_width);
}

/**
 * @brief Get next index
 *
 * Call this function for each pixel to get the sorted index.
 * Important: Before using this function call the "init" function.
 *
 * @return Sorted index to use for the input data
 */
uint32_t IteratorIndexIn::calcNextIndex()
{
    uint32_t indexToReturn = m_indexOut;

    if (!m_verticalFlip) // NOTE: option is inverted due to something in the Mojave design causing
                         // the image to appear vertical flipped (probably something in the lens).
    {   /*
         * Because of the inside-to-outside conversion sequence of the EPC660
         * a vertical flip is accomplished by simply using the other row in the
         * scan pair. The other row is either one ahead or behind depending on
         * whether we're in the top or bottom half.
         */
        if (m_y < m_halfHeight)
        {
            indexToReturn += m_width;
        }
        else
        {
            indexToReturn -= m_width;
        }
    }
    if (m_horizontalFlip)
    {   /*
         * A horizontal flip simply means changing the row's indexing
         * from representing columns 0 .. m_width to m_width .. 0.
         */
        const uint32_t offsetFromFront = indexToReturn % m_width;
        const int32_t hFlipOffset = m_width - 1 - 2 * offsetFromFront;
        indexToReturn += hFlipOffset;
    }

    m_x++; // next pixel in row
    if (m_x >= m_width) // overflow to next row
    {
        m_x = 0;
        m_y++;
        if (m_y == m_halfHeight)
        {
            m_addPerLineBreak = m_width + 1;
            m_indexOut++;
        }
        else
        {
            m_indexOut += m_addPerLineBreak;
        }
    }
    else
    {
        m_indexOut++;
    }

    return indexToReturn;
}

/**
 * @brief Get next index
 *
 * Call this function for each pixel to get the sorted index.
 * Important: Before using this function call the "init" function.
 *
 * @return sorted index
 */
uint32_t IteratorIndexIn::getNext()
{
    return calcNextIndex();
}

/**
 * @brief Get input index from output index
 *
 * This function directly returns the input index from the given output index. This is useful if only some pixels
 * are used(for example for broken pixels index) because the performance is not as good as when using the function "getNext" in a loop
 *
 * @param outputIndex Linear output index
 * @return sorted index
 */
uint32_t IteratorIndexIn::getIndex(const uint32_t outputIndex)
{
    uint32_t numPixelHalf = m_width * m_halfHeight;
    uint32_t index = 0;

    //Look at the unit test "TE_IteratorIndexIn". That helps a lot to understand.
    if (outputIndex >= numPixelHalf)
    {
        index = m_width + (((outputIndex - numPixelHalf) / m_width) * 2 * m_width)
                + (outputIndex % m_width);
    }
    else
    {
        index = m_indexOut - ((outputIndex / m_width) * 2 * m_width) + (outputIndex % m_width);
    }

    return index;
}

/** @} */
