#include <lenny/samp/PercentageRange.h>
#include <lenny/tools/Gui.h>
#include <lenny/tools/Logger.h>

namespace lenny::samp {

PercentageRange::PercentageRange(const std::pair<double, double>& values) : range({values.first, values.second}) {
    set(values);
}

std::pair<double, double> PercentageRange::get() const {
    return {range.first.get(), range.second.get()};
}

void PercentageRange::set(const std::pair<double, double>& values) {
    range.first.set(values.first);
    range.second.set(values.second);
    if (range.first.get() > range.second.get()) {
        LENNY_LOG_WARNING("Lower value is larger than upper value -> setting lower value to upper value")
        range.first.set(range.second.get());
    }
}

void PercentageRange::drawGui(const char* label) {
    using tools::Gui;
    Gui::I->Text(label);
    Gui::I->SameLine();
    if (range.first.drawGui("lower")) {
        if (range.first.get() > range.second.get())
            range.first.set(range.second.get());
    }
    Gui::I->SameLine();
    if (range.second.drawGui("upper")) {
        if (range.first.get() > range.second.get())
            range.second.set(range.first.get());
    }
}

}  // namespace lenny::samp