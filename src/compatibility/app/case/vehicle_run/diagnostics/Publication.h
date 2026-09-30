#pragma once
// Private implementation seam shared with qualification tests. Live TL access
// and the bounded fixture remain absent from the public FailureRun interface.
#include "FailureRun.h"
#include <functional>
#include "case/vehicle_startup/shell_execution/tests/self_contact/CandidateFailureFixture.h"
namespace crash::cases::vehicle_run::diagnostics::detail {
namespace fixture = vehicle_startup::shell_execution::self_contact_test;
// Host-only publication policy; this seam supplies no simulation authority.
struct CaptureEvidence {
    bool seen=false, complete=false, owners_equivalent=false;
    std::uint64_t accepted_epoch=0;
    tlfea::contact::SelfContactTransactionReport report;
    std::string error;
};
using ExportOperation=std::function<std::string(const std::filesystem::path&)>;
FailureReport PublishEvidence(const Result&,const CaptureEvidence&,
    const std::filesystem::path&,const ExportOperation&);
void CheckCapturedFailure(const Result&, const tlfea::contact::SelfContactTransactionReport&,
                          std::uint64_t accepted_epoch);
FailureReport PublishFailure(const Result&, const fixture::CandidateFailureFixture&,
                             const std::filesystem::path& destination);
} // namespace crash::cases::vehicle_run::diagnostics::detail
