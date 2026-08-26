#include "bumperbot_utils/safety_state.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

namespace
{
constexpr double kDangerDistance = 0.1;
constexpr double kWarningDistance = 0.5;
}

TEST(SafetyState, ClassifiesDangerThresholdBoundary)
{
    EXPECT_EQ(
        classifySafetyState({0.099F}, kDangerDistance, kWarningDistance),
        State::DANGER);
    EXPECT_EQ(
        classifySafetyState({0.1F}, kDangerDistance, kWarningDistance),
        State::DANGER);
    EXPECT_EQ(
        classifySafetyState({0.101F}, kDangerDistance, kWarningDistance),
        State::WARNING);
}

TEST(SafetyState, ClassifiesWarningThresholdBoundary)
{
    EXPECT_EQ(
        classifySafetyState({0.5F}, kDangerDistance, kWarningDistance),
        State::WARNING);
    EXPECT_EQ(
        classifySafetyState({0.501F}, kDangerDistance, kWarningDistance),
        State::FREE);
}

TEST(SafetyState, DangerReadingTakesPrecedence)
{
    EXPECT_EQ(
        classifySafetyState({1.0F, 0.3F, 0.05F}, kDangerDistance, kWarningDistance),
        State::DANGER);
}

TEST(SafetyState, IgnoresEmptyAndNonFiniteScans)
{
    EXPECT_EQ(
        classifySafetyState({}, kDangerDistance, kWarningDistance),
        State::FREE);
    EXPECT_EQ(
        classifySafetyState(
            {std::numeric_limits<float>::infinity(),
             -std::numeric_limits<float>::infinity(),
             std::numeric_limits<float>::quiet_NaN()},
            kDangerDistance,
            kWarningDistance),
        State::FREE);
}
