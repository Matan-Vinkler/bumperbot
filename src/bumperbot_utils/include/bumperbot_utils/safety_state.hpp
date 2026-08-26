#ifndef SAFETY_STATE_HPP_
#define SAFETY_STATE_HPP_

#include <vector>

enum State { FREE, WARNING, DANGER };

State classifySafetyState(
    const std::vector<float> & ranges,
    double danger_distance,
    double warning_distance);

#endif
