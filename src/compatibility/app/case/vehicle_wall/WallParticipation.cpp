#include "WallParticipation.h"
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_wall::detail {
tl::fea::ShellPhysicalScratchParticipationForecast ForecastWallParticipation(
    std::uint64_t source_id,const tl::fea::ShellPhysicalScratchParticipationLimits& limits) {
    tl::fea::ShellPhysicalScratchParticipation issuer;
    tl::fea::ShellPhysicalScratchParticipationForecast result;
    const auto report=tl::fea::ShellBatchPublication::ForecastPhysicalScratchParticipation(
        {{&issuer,source_id},{}},limits,result);
    output::Require(report.status==tl::fea::ShellPublicationStatus::Success,report.message);
    return result;
}
} // namespace crash::cases::vehicle_wall::detail
