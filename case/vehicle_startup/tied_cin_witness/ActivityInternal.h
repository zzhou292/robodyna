#pragma once
#include "ActivityValues.h"
namespace crash::cases::vehicle_startup {
struct TiedCinWitnessActivity::Impl {
    explicit Impl(const TiedCinWitnessRoster& value) : roster(value) {}
    TiedCinWitnessRoster roster;
    TiedCinActivityForecast forecast;
    std::vector<std::uint8_t> qeph,t3,qbat,candidate,accepted;
    tl::fea::NodalStamp stamp;
    TiedCinActivityReport receipt;
    bool valid=false,poisoned=false;
};
}
