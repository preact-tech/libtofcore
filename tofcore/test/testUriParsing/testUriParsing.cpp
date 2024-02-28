/**
 * @file testLoadCorrections.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 */
#include <gtest/gtest.h>
#include "uri.hpp"

using namespace std::string_literals;

TEST(testUriParsing, devicePath)
{
    {
        uri test_uri("http://foo.bar.com");
        EXPECT_EQ(test_uri.get_scheme(), "http") << "expected scheme to be http";
        EXPECT_EQ(test_uri.get_host(), "foo.bar.com") << "expected host to be foo.bar.com";
        EXPECT_EQ(test_uri.get_path(), "") << "expected path to be ''";
    }
    {
        uri test_uri("tofserial:/dev/ttyACM0?baudrate=19200&");
        EXPECT_EQ(test_uri.get_scheme(), "tofserial") << "expected scheme to be 'tofserial'";
        EXPECT_EQ(test_uri.get_host(), "") << "expected host to be ''";
        EXPECT_EQ(test_uri.get_path(), "/dev/ttyACM0") << "expected path to be '/dev/ttyACM0'";
        EXPECT_TRUE(test_uri.is_rooted()) << "expected path to be rooted";
        auto query_dict = test_uri.get_query_dictionary();
        EXPECT_EQ(query_dict["baudrate"], "19200");
    }
    {
        uri test_uri("tofserial:COM1?&baudrate=115200");
        EXPECT_EQ(test_uri.get_scheme(), "tofserial") << "expected scheme to be 'dev'";
        EXPECT_EQ(test_uri.get_host(), "") << "unexpected host value";
        EXPECT_EQ(test_uri.get_path(), "COM1") << "unexpected path value";
        auto query_dict = test_uri.get_query_dictionary();
        EXPECT_EQ(query_dict["baudrate"], "115200");
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
        try
        {
            uri test_uri("COM1", uri::scheme_category::Hierarchical);
            FAIL() << "Expected an exception to be thrown";
        }
        catch(const std::invalid_argument& err)
        {
            EXPECT_EQ(err.what(), "End of URI found while parsing the scheme. Supplied URI was: \"COM1\"."s);
        }
        catch(...)
        {
            FAIL() << "expected std::invalid_argument exception";
        }
    }

}

