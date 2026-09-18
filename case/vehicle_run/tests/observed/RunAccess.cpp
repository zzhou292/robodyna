#include "RunAccess.h"
#include "Contribution.h"
#include "case/vehicle_run/RunState.h"
#include "case/vehicle_startup/shell_execution/tests/self_contact/CandidateCapture.h"
#include "lib_utils/BoundedArena.h"
#include "output/ArtifactIO.h"
#include <initializer_list>

namespace crash::cases::vehicle_run::observed {
namespace {
struct Installation { Observer observer; };
static_assert(sizeof(Installation) + sizeof(ObservedContribution) <= ContributionHostReserve);
static_assert(sizeof(fixture::CandidateFailureFixture) <= fixture::FailureCaptureHostCap);

void Install(void* context, vehicle_dynamics::VehiclePhysicalDynamics& dynamics) {
    const auto& installation = *static_cast<const Installation*>(context);
    vehicle_self_contact::CandidateRigidCouponAccess::InstallFailureObserver(
        dynamics, installation.observer);
}
std::size_t Sum(std::size_t cap, std::initializer_list<std::size_t> sizes) {
    tl::util::BoundedArenaLayout arena(cap);
    tl::util::ArenaRegion region;
    for (const auto bytes : sizes)
        output::Require(arena.Append<std::byte>(bytes, region),
                        "Observed qualification reservation exceeds the existing run cap");
    return arena.bytes();
}
}

Reservation Preflight(const Forecast& forecast) {
    output::Require(forecast.contact.self_contact.has_value(),
                    "Observed qualification requires the explicit self-contact profile");
    return {
        Sum(forecast.caps.host_bytes, {forecast.complete_host_bytes,
            fixture::FailureCaptureHostCap, ContributionHostReserve}),
        Sum(forecast.caps.archive_bytes, {forecast.complete_archive_bytes,
            fixture::FailureFixtureArchiveCap})};
}

ObservedResult RunAccess::Execute(
    const PreparedRun& run, const std::filesystem::path& destination,
    const Control& control) {
    ObservedResult result;
    result.reservation = Preflight(run.forecast());
    result.failure = std::make_unique<fixture::CandidateFailureFixture>(
        run.data_->contact.runtime_limits().self_contact.transaction);
    Installation installation{result.failure->observer()};
    result.run = run.ExecuteImpl(destination, control, &installation, &Install);
    return result;
}

} // namespace crash::cases::vehicle_run::observed
