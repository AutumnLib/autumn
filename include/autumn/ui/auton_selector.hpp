#pragma once

#include <functional>
#include <string>
#include <vector>

namespace autumn {

class AutonSelector {
public:
    void addRoutine(const std::string& name, std::function<void()> routine);
    void runSelected(int index);

    void showLlemuMenu() const;
    int selectedIndex() const { return selectedIndex_; }
    void setSelectedIndex(int index);

private:
    std::vector<std::string> names_;
    std::vector<std::function<void()>> routines_;
    int selectedIndex_ = 0;
};

} // namespace autumn
