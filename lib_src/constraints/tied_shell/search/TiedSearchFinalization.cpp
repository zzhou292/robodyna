#include "FinalizationInternal.h"
#include <new>
namespace tl::constraints::tied_shell {
FinalizationReport FinalizeSearch(const FinalizationInput& input, FinalizedSearch* output,
    FinalizationLimits limits) noexcept {
  using namespace finalization_detail;
  if (!output) return Fail(FinalizationStatus::InvalidInput,"Missing finalization output");
  try {
    std::size_t bytes = 0;
    auto report = Preflight(input,*output,limits,bytes);
    if (!report) return report;
    report = Check(input);
    if (!report) return report;
    auto draft = std::make_shared<FinalizationMaps>();
    Build(input,*draft);
    draft->owned_payload_bytes = Owned(*draft);
    draft->startup_payload_bytes = bytes;
    std::shared_ptr<const FinalizationMaps> published = std::move(draft);
    output->data_.swap(published);
    return {};
  } catch (const std::bad_alloc&) {
    return Fail(FinalizationStatus::AllocationFailure,"Finalization allocation failed");
  }
}
}
