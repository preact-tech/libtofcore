#ifndef __MOCK_CONNECTION_H_
#define __MOCK_CONNECTION_H_
/**
 * @file mock_connection.hpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Mock of the serial or IP interface
 * for unit testing.
 */

#include <gmock/gmock.h>
#include "connection.hpp"
#include "span.hpp"

namespace tofcore {
class MockConnection : public Connection_T
{
public:
    MOCK_METHOD(void, send, (uint16_t command, const std::vector<ScatterGatherElement> &data),(override));
    MOCK_METHOD(void, send, (uint16_t command, const uint8_t *data, uint32_t size),(override));
    MOCK_METHOD(void, send, (uint16_t command, const std::vector<uint8_t> &buf),(override));
    MOCK_METHOD(std::optional<std::vector<std::byte>>, send_receive, (uint16_t command,
        const std::vector<ScatterGatherElement> &data, std::chrono::steady_clock::duration timeout),(override));
    MOCK_METHOD(std::optional<std::vector<std::byte>>, send_receive,(uint16_t command, const std::vector<uint8_t> &buf,
        std::chrono::steady_clock::duration timeout) ,(override));
    MOCK_METHOD(std::optional<std::vector<std::byte>>, send_receive,(uint16_t command, const uint8_t *data, uint32_t size,
                                                         std::chrono::steady_clock::duration timeout) ,(override));
    MOCK_METHOD(void, reset_parser, (), (override));
    MOCK_METHOD(void, subscribe,(on_measurement_callback_t callback), (override));
};
} // end namespace tofcore
#endif // __MOCK_CONNECTION_H_
