#ifndef CRC32_H
#   define CRC32_H
/**
 * @file crc32
 *
 * Copyright 2021 PreAct Technologies
 *
 */
#   include <stdint.h>

uint32_t calcCrc32(const uint8_t *buffer, unsigned int length);
uint32_t updateCrc32(uint32_t crc, const uint8_t *buffer, unsigned int length);

#endif // CRC32_H
