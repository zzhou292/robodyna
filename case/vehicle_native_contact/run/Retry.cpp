#include "State.h"
#include "case/vehicle_dynamics/native_contact/Group.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
namespace crash::cases::vehicle_native_contact {
namespace {
bool SameBits(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) if (output::Bits(a[i]) != output::Bits(b[i])) return false;
    return true;
}
bool Same(const n::TransactionDiagnostics& a, const n::TransactionDiagnostics& b) {
    return a.raw_candidates == b.raw_candidates && a.optimized_candidates == b.optimized_candidates &&
        a.kept_occurrences == b.kept_occurrences && a.active_forces == b.active_forces &&
        a.reference_rebuilt == b.reference_rebuilt && output::Bits(a.elastic_energy) == output::Bits(b.elastic_energy) &&
        output::Bits(a.damping_work) == output::Bits(b.damping_work) && output::Bits(a.friction_work) == output::Bits(b.friction_work);
}
bool Same(const tl::fea::NativeContactPublicationSnapshot& a, const tl::fea::NativeContactPublicationSnapshot& b) {
    return a.available == b.available && a.force_phase_available == b.force_phase_available &&
        a.generation == b.generation && a.selectors.history == b.selectors.history &&
        a.selectors.reference == b.selectors.reference && a.selectors.reference_generation == b.selectors.reference_generation &&
        a.selectors.has_reference == b.selectors.has_reference && tl::fea::trial_identity::SameStamp(a.stamp, b.stamp) &&
        tl::fea::trial_identity::SameStamp(a.force_base_stamp, b.force_base_stamp);
}
}
void PreparedRun::Session::VerifyInitialRetry() {
    output::Require(source.config.verify_initial_retry && source.forecast.retry_bytes, "Retry probe was not forecast");
    Capture();
    const auto& producer = capture.frames();
    const auto initial = dynamics.accepted();
    const auto positions = producer.frame()->position_xyz;
    const auto plastic = producer.frame()->plastic_points;
    const auto activity = producer.activity()->words();
    output::Require((positions.capacity() + plastic.capacity()) * sizeof(double) + activity.capacity() * sizeof(std::uint64_t) <=
                        source.forecast.retry_bytes, "Actual retry snapshot exceeds its reservation");
    const auto* group = dynamics.native_contact_group();
    std::array<tl::fea::NativeContactPublicationSnapshot, 2> published;
    for (std::size_t i = 0; i < published.size(); ++i) published[i] = group->transaction(i).accepted();
    const auto check = [&] {
        output::Require(tl::fea::trial_identity::SameStamp(initial, dynamics.accepted()), "Private attempt changed accepted owner");
        Capture();
        output::Require(SameBits(producer.frame()->position_xyz, positions) && SameBits(producer.frame()->plastic_points, plastic) &&
                            producer.activity()->words() == activity && producer.frame()->stamp.epoch == 0,
                        "Private attempt changed accepted physical frame/activity");
        for (std::size_t i = 0; i < published.size(); ++i)
            output::Require(Same(published[i], group->transaction(i).accepted()), "Private attempt changed accepted native publication");
    };
    try {
        const auto first = dynamics.PrepareStep().native_contact;
        check();
        dynamics.DiscardStep();
        check();
        const auto retry = dynamics.PrepareStep().native_contact;
        output::Require(first.count == retry.count && retry.count == 2, "Retry interface roster changed");
        for (std::size_t i = 0; i < retry.count; ++i)
            output::Require(first.roles[i] == retry.roles[i] && Same(first.interfaces[i].diagnostics, retry.interfaces[i].diagnostics),
                            "Actual discarded native attempt differs from its retry");
        dynamics.DiscardStep();
        check();
    } catch (...) {
        dynamics.DiscardStep();
        throw;
    }
}
} // namespace crash::cases::vehicle_native_contact
