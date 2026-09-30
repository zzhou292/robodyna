#pragma once
#include "SampledShellPlasticityTotals.h"
#include "output/ArtifactIO.h"
#include <iosfwd>
#include <utility>

namespace crash::output::full_shell {
class Context;
struct FrameRecord;
}

namespace crash::cases::vehicle_run::detail {

// The caller supplies its existing authenticated capture context/frame. This
// validates and summarizes values; it does not authenticate arbitrary histories.
SampledShellPlasticityTotals SummarizeShellSample(const SampledShellPlasticityTotals&,
    const output::full_shell::Context&, const output::full_shell::FrameRecord&);
output::Document SampledShellPlasticityDocument(const SampledShellPlasticityTotals&);
void WriteSampledShellPlasticityProgress(std::ostream&, const SampledShellPlasticityTotals&);

// The callback must persist this same frame/activity pair through the existing
// archive. A validation or partial write failure leaves public summary unchanged.
// Assignment of the staged scalar value after a successful write cannot fail.
template<class WriteSample>
void SaveShellSample(SampledShellPlasticityTotals& totals,
    const output::full_shell::Context& context, const output::full_shell::FrameRecord& frame,
    WriteSample&& write_sample) {
    const auto next = SummarizeShellSample(totals, context, frame);
    std::forward<WriteSample>(write_sample)();
    totals = next;
}

} // namespace crash::cases::vehicle_run::detail
