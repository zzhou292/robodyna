#pragma once

#include "GuidedPlateCase.h"
#include "output/CsvLedgerSegments.h"
#include <array>

namespace crash::case_data {
inline constexpr std::array<const char*,3> kGuidedIntervalFiles{
    "accepted-intervals.csv","shell-intervals.csv","contact-intervals.csv"};
inline constexpr std::size_t kGuidedFieldFileCap=128*1024, kGuidedMeshFileCap=64*1024,
                             kGuidedObjFileCap=16*1024, kGuidedStaticReserve=8*1024*1024;
struct GuidedPlateOutputForecast {
    std::array<std::size_t,3> ledger_bytes{};
    std::array<output::CsvLedgerPlan,3> ledgers;
    std::size_t frames=0,total_bytes=0;
    bool segmented=false;
};
// Checked encoded worst-case sizes, before directory creation or stepping.
// Throws on overflow/cap violation. Exactly scheduled initial/final frames.
GuidedPlateOutputForecast ForecastGuidedPlateOutput(std::uint64_t required_steps,unsigned frame_every);
const std::array<std::string,3>& GuidedPlateIntervalHeaders();
std::array<std::string,3> GuidedPlateIntervalRows(const tl::fea::NodalStamp& base,const GuidedPlateMetrics&);
} // namespace crash::case_data
