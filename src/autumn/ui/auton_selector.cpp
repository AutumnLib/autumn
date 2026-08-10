#include "autumn/ui/auton_selector.hpp"

#include "liblvgl/lvgl.h"
#include "pros/llemu.hpp"

#include <cstdint>
#include <cstdio>

namespace autumn {

namespace {
const std::string kEmptyName{};

void onRouteClicked(lv_event_t* event) {
    auto* selector = static_cast<AutonSelector*>(lv_event_get_user_data(event));
    auto* button = static_cast<lv_obj_t*>(lv_event_get_target(event));
    if (selector == nullptr || button == nullptr) {
        return;
    }
    const auto index = static_cast<int>(reinterpret_cast<std::intptr_t>(lv_obj_get_user_data(button)));
    selector->setSelectedIndex(index);
}
} // namespace

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

void AutonSelector::showLvglMenu() {
    // Idempotent: never build a second list on the same screen.
    if (list_ != nullptr) {
        return;
    }

    lv_obj_t* screen = lv_screen_active();
    lv_obj_clean(screen);

    statusLabel_ = lv_label_create(screen);
    lv_label_set_text(statusLabel_, "Autumn Auton Selector");
    lv_obj_align(statusLabel_, LV_ALIGN_TOP_MID, 0, 4);

    poseLabel_ = lv_label_create(screen);
    lv_label_set_text(poseLabel_, "X: -- Y: -- Theta: --");
    lv_obj_align(poseLabel_, LV_ALIGN_TOP_MID, 0, 28);

    list_ = lv_list_create(screen);
    lv_obj_set_size(list_, 460, 180);
    lv_obj_align(list_, LV_ALIGN_BOTTOM_MID, 0, -8);

    for (std::size_t i = 0; i < names_.size(); ++i) {
        lv_obj_t* button = lv_list_add_button(list_, nullptr, names_[i].c_str());
        lv_obj_set_user_data(button, reinterpret_cast<void*>(static_cast<std::intptr_t>(i)));
        lv_obj_add_event_cb(button, onRouteClicked, LV_EVENT_CLICKED, this);
    }
}

void AutonSelector::setPoseText(const char* text) {
    if (poseLabel_ != nullptr && text != nullptr) {
        lv_label_set_text(poseLabel_, text);
    }
}

void AutonSelector::setSelectedIndex(int index) {
    if (index >= 0 && index < static_cast<int>(names_.size())) {
        selectedIndex_ = index;
        if (statusLabel_ != nullptr) {
            const std::string text = "Selected: " + names_[static_cast<std::size_t>(index)];
            lv_label_set_text(statusLabel_, text.c_str());
        }
    }
}

const std::string& AutonSelector::selectedName() const {
    if (selectedIndex_ < 0 || selectedIndex_ >= static_cast<int>(names_.size())) {
        return kEmptyName;
    }
    return names_[static_cast<std::size_t>(selectedIndex_)];
}

} // namespace autumn
