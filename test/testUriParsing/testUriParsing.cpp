/**
 * @file testLoadCorrections.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 */
#include <gtest/gtest.h>
#include "uri.hpp"

TEST(testUriParsing, devicePath)
{
    {
        uri test_uri("http://foo.bar.com");
        EXPECT_EQ(test_uri.get_scheme(), "http") << "expected scheme to be http";
        EXPECT_EQ(test_uri.get_host(), "foo.bar.com") << "expected host to be foo.bar.com";
        EXPECT_EQ(test_uri.get_path(), "") << "expected path to be ''";
    }
    {
        uri test_uri("device:/dev/ttyACM0?baudrate=19200&protocol_version=1");
        EXPECT_EQ(test_uri.get_scheme(), "device") << "expected scheme to be 'dev'";
        EXPECT_EQ(test_uri.get_host(), "") << "expected host to be ''";
        EXPECT_EQ(test_uri.get_path(), "dev/ttyACM0") << "expected path to be 'dev/ttyACM0'";
        EXPECT_TRUE(test_uri.is_rooted()) << "expected path to be rooted";
        auto query_dict = test_uri.get_query_dictionary();
        EXPECT_EQ(query_dict["baudrate"], "19200");
        EXPECT_EQ(query_dict["protocol_version"], "1");
    }
    {
        uri test_uri("device:COM1?protocol_version=1&baudrate=115200");
        EXPECT_EQ(test_uri.get_scheme(), "device") << "expected scheme to be 'dev'";
        EXPECT_EQ(test_uri.get_host(), "") << "unexpected host value";
        EXPECT_EQ(test_uri.get_path(), "COM1") << "unexpected path value";
        auto query_dict = test_uri.get_query_dictionary();
        EXPECT_EQ(query_dict["baudrate"], "115200");
        EXPECT_EQ(query_dict["protocol_version"], "1");
        EXPECT_FALSE(test_uri.is_rooted()) << "expected path should not be rooted";
    }
    {
        uri test_uri("tofnet://10.10.31.180:5353?modulation_frequency=1200");
        EXPECT_EQ(test_uri.get_scheme(), "tofnet") << "unexpected scheme";
        EXPECT_EQ(test_uri.get_host(), "10.10.31.180") << "unexpected host value";
        EXPECT_EQ(test_uri.get_port(), 5353) << "unexpected port value";
        EXPECT_EQ(test_uri.get_path(), "") << "unexpected path value";
        auto query_dict = test_uri.get_query_dictionary();
        EXPECT_EQ(query_dict["modulation_frequency"], "1200");
    }

    {
        uri test_uri("COM1", uri::scheme_category::Hierarchical);
        EXPECT_EQ(test_uri.get_scheme(), "") << "unexpected scheme";
        EXPECT_EQ(test_uri.get_host(), "") << "unexpected host value";
        EXPECT_EQ(test_uri.get_port(), 0) << "unexpected port value";
        EXPECT_EQ(test_uri.get_path(), "") << "unexpected path value";
    }

}

