#ifndef PO_COUNT_HPP
#   define PO_COUNT_HPP
/**
 * @file po_count.hpp
 *
 * Copyright 2024 PreAct Technologies
 *
 * Class to count boost program options
 */
#include <boost/program_options.hpp>

namespace test
{

namespace po = boost::program_options;

class CountValue: public po::typed_value<std::uint32_t>
{
public:
    CountValue() : CountValue(nullptr)
    {
    }

    CountValue(std::uint32_t *store) :
                po::typed_value<std::uint32_t>(store)
    {
        // Ensure that no tokens may be passed as a value.
        default_value(0);
        zero_tokens();
    }

private:

    virtual void xparse(boost::any &store, const std::vector<std::string>& /*tokens*/) const
    {
        // Replace the stored value with the access count.
        store = boost::any(++count_);
    }

    mutable std::uint32_t count_ { 0 };
};

} // namespace test

#endif // PO_COUNT_HPP


