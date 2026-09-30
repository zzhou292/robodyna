#pragma once
#include "../TiedCinWitnessActivity.h"
namespace crash::cases::vehicle_startup::cin_witness_detail {
struct FamilyActivity {
    const std::uint8_t* qeph = nullptr;
    const std::uint8_t* t3 = nullptr;
    const std::uint8_t* qbat = nullptr;
    std::size_t qeph_count = 0, t3_count = 0, qbat_count = 0;
};
// Value-only mapping of already validated family bytes; not an activity source.
// All late input checks precede output. Pending is a complete valid mapping.
TiedCinActivityReport MapActivity(const TiedCinWitnessData&,FamilyActivity,std::uint8_t*,std::size_t);
TiedCinActivityForecast ActivityBudget(std::size_t retained,std::size_t parents,
    std::size_t witnesses,std::size_t fixed,TiedCinActivityLimits);
}
