#include "DiurnalCycle.h"

#include <algorithm>
#include <cmath>

namespace Parapenting::Physics
{
namespace
{
constexpr double Pi = 3.14159265358979323846;

double SmoothStep01(double value)
{
    const double t = std::clamp(value, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}
}

double WrapLocalHour(double hour)
{
    double wrapped = std::fmod(hour, 24.0);
    if (wrapped < 0.0) wrapped += 24.0;
    return wrapped;
}

DiurnalState EvaluateDiurnalCycle(
    double startLocalHour, double simulationTimeSeconds,
    double simulatedSecondsPerLocalHour)
{
    const double secondsPerHour =
        std::max(1.0, simulatedSecondsPerLocalHour);
    const double hour = WrapLocalHour(
        startLocalHour + std::max(0.0, simulationTimeSeconds)
            / secondsPerHour);

    constexpr double SunriseHour = 6.25;
    constexpr double SunsetHour = 20.25;
    constexpr double DayLength = SunsetHour - SunriseHour;
    const bool daytime = hour >= SunriseHour && hour <= SunsetHour;
    const double dayPhase = std::clamp(
        (hour - SunriseHour) / DayLength, 0.0, 1.0);
    const double solarSine = daytime ? std::sin(dayPhase * Pi) : 0.0;
    const double sunElevation = daytime
        ? 62.0 * solarSine
        : -14.0 * std::sin(
            Pi * std::clamp(
                hour > SunsetHour
                    ? (hour - SunsetHour) / (24.0 - SunsetHour + SunriseHour)
                    : (hour + 24.0 - SunsetHour)
                        / (24.0 - SunsetHour + SunriseHour),
                0.0, 1.0));
    const double sunAzimuth = daytime
        ? 72.0 + 216.0 * dayPhase
        : 288.0 + 144.0 * std::clamp(
            hour > SunsetHour
                ? (hour - SunsetHour) / (24.0 - SunsetHour + SunriseHour)
                : (hour + 24.0 - SunsetHour)
                    / (24.0 - SunsetHour + SunriseHour),
            0.0, 1.0);

    // Ground heat flux lags solar noon. Positive heating builds after sunrise
    // and persists into early evening; drainage strengthens after sunset and
    // toward dawn.
    const double heatingRise = SmoothStep01((hour - 7.3) / 3.0);
    const double heatingFall = 1.0 - SmoothStep01((hour - 18.0) / 2.7);
    const double positiveHeating =
        std::max(0.0, heatingRise * heatingFall);
    double negativeHeating = 0.0;
    if (hour >= 19.0)
        negativeHeating = SmoothStep01((hour - 19.0) / 3.0);
    else if (hour <= 7.5)
        negativeHeating = 1.0 - SmoothStep01((hour - 4.5) / 3.0);
    const double signedHeating =
        positiveHeating - 0.72 * negativeHeating;

    const double convective = positiveHeating
        * SmoothStep01((sunElevation - 5.0) / 32.0);
    const double ambient = std::clamp(
        0.08 + 0.92 * SmoothStep01((sunElevation + 5.0) / 25.0),
        0.08, 1.0);
    const double warm = std::clamp(
        1.0 - std::abs(sunElevation - 7.0) / 16.0, 0.0, 1.0);

    return {
        hour,
        sunElevation,
        WrapLocalHour(sunAzimuth / 15.0) * 15.0,
        signedHeating,
        convective,
        ambient,
        warm
    };
}
}
