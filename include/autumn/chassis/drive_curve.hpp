#pragma once

#include "autumn/util/units.hpp"

namespace autumn {

// Exponential joystick curve. Input and output are on the 0-127 joystick scale.
//
// Deliberate divergence from LemLib 0.5.6: this maps the full stick to full
// scale so curve(127) == 127. LemLib's ExpoDriveCurve leaves the output near
// minOutput even at full deflection (about 11 of 127 with the default
// arguments), which teams worked around by scaling downstream. The deadband and
// minOutput parameters mean the same thing as in LemLib; only the shape between
// them differs, and the response is slightly softer than LemLib's for the same
// `curve` value. See docs/index.md before copying LemLib tuning notes.
class ExpoDriveCurve {
public:
    ExpoDriveCurve(int deadband = 3, int minOutput = 10, double curve = 1.019)
        : deadband_(deadband), minOutput_(minOutput), curve_(curve) {}

    double curve(double input) const;

private:
    int deadband_;
    int minOutput_;
    double curve_;
};

} // namespace autumn
