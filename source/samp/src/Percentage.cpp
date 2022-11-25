#include <lenny/samp/Percentage.h>
#include <lenny/tools/Gui.h>
#include <lenny/tools/Logger.h>

namespace lenny::samp {

Percentage::Percentage(const double& value) {
    set(value);
}

double Percentage::get() const {
    return this->value;
}

void Percentage::set(const double& val) {
    if (val < 0.0 || val > 1.0)
        LENNY_LOG_WARNING("Trying to set value outside of bounds: %lf -> bounding to range...")
    this->value = val;
    tools::utils::boundToRange(this->value, 0.0, 1.0);
}

bool Percentage::drawGui(const char* label) {
    return tools::Gui::I->Slider(label, this->value, 0.0, 1.0);
}

}  // namespace lenny::samp