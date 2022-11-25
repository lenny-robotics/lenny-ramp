#pragma once

namespace lenny::samp {

class Percentage {
public:
    Percentage(const double& value);
    ~Percentage() = default;

    double get() const;
    void set(const double& val);

    bool drawGui(const char* label);

private:
    double value;  //Between 0 and 1
};

}  // namespace lenny::samp