#pragma once

#include <lenny/samp/Percentage.h>

#include <utility>

namespace lenny::samp {

class PercentageRange {
public:
    PercentageRange(const std::pair<double, double>& values);
    ~PercentageRange() = default;

    std::pair<double, double> get() const;
    void set(const std::pair<double, double>& values);

    void drawGui(const char* label);

private:
    std::pair<Percentage, Percentage> range;
};

}  // namespace lenny::samp