#include "tied_cin_witness/ActivityInternal.h"
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_startup {
TiedCinActivityForecast TiedCinWitnessActivity::Forecast(const TiedCinWitnessRoster& roster,TiedCinActivityLimits limits) {
    return cin_witness_detail::ActivityBudget(roster.forecast().total_host_bytes,
        roster.data().counts.source_parents,roster.data().counts.witnesses,
        sizeof(Impl)+sizeof(TiedCinWitnessActivity),limits);
}
std::unique_ptr<TiedCinWitnessActivity> TiedCinWitnessActivity::Create(const TiedCinWitnessRoster& roster,
        TiedCinActivityLimits limits) {
    const auto forecast=Forecast(roster,limits);
    auto next=std::make_unique<Impl>(roster);
    next->forecast=forecast;
    const auto& binding=roster.shells();
    next->qeph.resize(binding.qeph_count());
    next->t3.resize(binding.t3_count());
    next->qbat.resize(binding.qbat_count());
    next->candidate.resize(roster.data().counts.witnesses);
    next->accepted.resize(roster.data().counts.witnesses);
    std::size_t actual=sizeof(Impl)+sizeof(TiedCinWitnessActivity);
    for (const auto* array:{&next->qeph,&next->t3,&next->qbat,&next->candidate,&next->accepted}) {
        output::Require(array->capacity()<=forecast.workspace_bytes-actual,"Actual CIN activity workspace exceeds preflight");
        actual+=array->capacity();
    }
    return std::unique_ptr<TiedCinWitnessActivity>(new TiedCinWitnessActivity(std::move(next)));
}
TiedCinWitnessActivity::TiedCinWitnessActivity(std::unique_ptr<Impl> value) : impl_(std::move(value)) {}
TiedCinWitnessActivity::~TiedCinWitnessActivity()=default;
bool TiedCinWitnessActivity::has_accepted_activity() const noexcept { return impl_->valid; }
const tl::fea::NodalStamp& TiedCinWitnessActivity::accepted_stamp() const noexcept { return impl_->stamp; }
native_search::ClassificationView<std::uint8_t> TiedCinWitnessActivity::accepted_flags() const noexcept {
    return impl_->valid ? native_search::ClassificationView<std::uint8_t>{impl_->accepted.data(),impl_->accepted.size()} :
                          native_search::ClassificationView<std::uint8_t>{};
}
const TiedCinActivityForecast& TiedCinWitnessActivity::forecast() const noexcept { return impl_->forecast; }
}
