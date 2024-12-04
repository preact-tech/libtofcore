/**
 * @file testStreamingStateCommands.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 */
#include "mock_connection.hpp"
#include "connection.hpp"
#include "tof_sensor.hpp"
#include "TofCommand_IF.hpp"
#include <gtest/gtest.h>
#include <gmock/gmock.h>

using ::testing::Exactly;
using ::testing::Return;
using ::testing::An;
using ::testing::_;

using namespace tofcore;
using namespace TofComm;

using ReceiveBufferType = const std::vector<Connection_T::ScatterGatherElement>&;

TEST(testStreamingStateCommands, commandStopStream)
{
    std::unique_ptr<MockConnection> pConn = std::make_unique<MockConnection>();

    EXPECT_CALL(*pConn,
        subscribe(An<Connection_T::on_measurement_callback_t>()))
        .Times(Exactly(1))
        .WillOnce(Return()); 

    EXPECT_CALL(*pConn,
        send_receive(COMMAND_STOP_STREAM,An<ReceiveBufferType>(), _))
        .WillOnce(Return(std::optional<std::vector<std::byte>>{0}));

    Sensor uit(std::move(pConn));
    EXPECT_TRUE(uit.stopStream());
}

TEST(testStreamingStateCommands, commandGetStreamingStateNone)
{
    std::unique_ptr<MockConnection> pConn = std::make_unique<MockConnection>();

    EXPECT_CALL(*pConn,
        subscribe(An<Connection_T::on_measurement_callback_t>()))
        .Times(Exactly(1))
        .WillOnce(Return()); 

    EXPECT_CALL(*pConn,
        send_receive(COMMAND_GET_STREAMING_STATE,An<ReceiveBufferType>(), _))
        .WillOnce(Return(std::nullopt));

    Sensor uit(std::move(pConn));
    EXPECT_FALSE(uit.getSensorControlState().has_value());
}

TEST(testStreamingStateCommands, commandGetTofStateValues)
{
    std::vector<SensorControlStatus> expectedStates = {
        SensorControlStatus::IDLE,               ///< No ToF activity
        SensorControlStatus::CAPTURE,            ///< Capturing data internally
        SensorControlStatus::SEND,               ///< Sending a single result
        SensorControlStatus::STREAM,             ///< Continually streaming data
        SensorControlStatus::OVERTEMPERATURE,    ///< Disabled due to temperature
        SensorControlStatus::ERROR               ///< ToF unavailable due to an internal error
    };

    for (SensorControlStatus streaming_state: expectedStates)
    {
        const uint8_t state_code = static_cast<uint8_t>(streaming_state);
        std::byte state_byte{state_code};
        std::vector<std::byte> mock_response{state_byte};
        std::optional<std::vector<std::byte>> mock_data = mock_response;

        std::unique_ptr<MockConnection> pConn = std::make_unique<MockConnection>();
        EXPECT_CALL(*pConn,
            subscribe(An<Connection_T::on_measurement_callback_t>()))
            .Times(Exactly(1))
            .WillOnce(Return()); 

        EXPECT_CALL(*pConn,
            send_receive(COMMAND_GET_STREAMING_STATE,An<ReceiveBufferType>(), _))
            .WillOnce(Return(mock_data));

        Sensor uit(std::move(pConn));

        const std::optional<SensorControlStatus>& test_value = uit.getSensorControlState();
        EXPECT_EQ(test_value.value(), streaming_state); 
    }
}
