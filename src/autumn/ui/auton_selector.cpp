#include "autumn/ui/auton_selector.hpp"

#include "pros/llemu.hpp"

#include <cstdio>

namespace autumn {

void AutonSelector::addRoutine(const std::string& name, std::function<void()> routine) {
    names_.push_back(name);
    routines_.push_back(std::move(routine));
}

void AutonSelector::runSelected(int index) {
    if (index < 0 || index >= static_cast<int>(routines_.size())) {
        return;
    }
    selectedIndex_ = index;
    routines_[static_cast<std::size_t>(index)]();
}

void AutonSelector::showLlemuMenu() const {
    pros::lcd::clear();
    pros::lcd::set_text(0, "Autumn Auton Selector");
    for (std::size_t i = 0; i < names_.size() && i < 6; ++i) {
        char line[32];
        std::snprintf(line, sizeof(line), "%c %s", (static_cast<int>(i) == selectedIndex_) ? '>' : ' ',
                      names_[i].c_str());
        pros::lcd::set_text(static_cast<std::int32_t>(i + 1), line);
    }
}

void AutonSelector::setSelectedIndex(int index) {
    if (index >= 0 && index < static_cast<int>(names_.size())) {
        selectedIndex_ = index;
    }
}

} // namespace autumn
