#ifndef DBG_OUT_HPP
#   define DBG_OUT_HPP
/**
 * @file dbg_out.hpp
 *
 * Copyright 2024 PreAct Technologies
 *
 * Debug output classes for tests
 */
#include <iostream>

namespace test
{

class DebugOutput
{
public:
    DebugOutput() = default;
    DebugOutput(DebugOutput const&) = default;
    DebugOutput& operator =(DebugOutput const&) = delete;
    DebugOutput& operator =(DebugOutput&&) = delete;
    ~DebugOutput() = default;

    template<typename T>
    DebugOutput operator<<(const T &x)
    {
        if (!quiet) std::cout << x;
        return *this;
    }
    bool quiet { false };
};

class ErrorOutput
{
public:
    ErrorOutput() = default;
    ErrorOutput(ErrorOutput const&) = default;
    ErrorOutput& operator =(ErrorOutput const&) = delete;
    ErrorOutput& operator =(ErrorOutput&&) = delete;
    ~ErrorOutput() = default;

    template<typename T>
    ErrorOutput operator<<(const T &x)
    {
        std::cerr << x;
        return *this;
    }
};

} // namespace test

#endif // DBG_OUT_HPP


