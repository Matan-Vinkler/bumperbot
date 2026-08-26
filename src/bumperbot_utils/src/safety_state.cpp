#include "bumperbot_utils/safety_state.hpp"

#include <cmath>

State classifySafetyState(
    const std::vector<float> & ranges,
    double danger_distance,
    double warning_distance)
{
    const float danger_threshold = static_cast<float>(danger_distance);
    const float warning_threshold = static_cast<float>(warning_distance);

    State state = State::FREE;
    for (const float range_value : ranges)
    {
        if (!std::isfinite(range_value) || range_value > warning_threshold)
        {
            continue;
        }

        state = State::WARNING;
        if (range_value <= danger_threshold)
        {
            return State::DANGER;
        }
    }

    return state;
}
