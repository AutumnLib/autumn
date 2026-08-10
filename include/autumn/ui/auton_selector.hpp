#pragma once

#include <functional>
#include <string>
#include <vector>

// Forward-declare LVGL types so the header stays free of lvgl.h for LLEMU-only users.
struct _lv_obj_t;
typedef _lv_obj_t lv_obj_t;

namespace autumn {

class AutonSelector {
public:
    void addRoutine(const std::string& name, std::function<void()> routine);
    void runSelected(int index);

    // liblvgl touch menu (preferred). Idempotent — second call is a no-op.
    // Also creates a pose label updated via setPoseText().
    void showLvglMenu();

    // LLEMU text menu kept as a documented fallback (do not mix with LVGL).
    void showLlemuMenu() const;

    // Update the LVGL pose status label (no-op if menu not shown).
    void setPoseText(const char* text);

    int selectedIndex() const { return selectedIndex_; }
    void setSelectedIndex(int index);
    const std::string& selectedName() const;

private:
    std::vector<std::string> names_;
    std::vector<std::function<void()>> routines_;
    int selectedIndex_ = 0;
    lv_obj_t* list_ = nullptr;
    lv_obj_t* poseLabel_ = nullptr;
    lv_obj_t* statusLabel_ = nullptr;
};

} // namespace autumn
