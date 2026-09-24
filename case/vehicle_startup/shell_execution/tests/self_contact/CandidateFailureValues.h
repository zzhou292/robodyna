#pragma once
#include "CandidateFailureFixture.h"

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test::failure_detail {
// Read explicit nonphysical provenance from v2, or absent=false from v1.
// Consume only the validated v2 key in this owned document so the existing
// exact common-field validator can reject all other extra/missing keys.
bool ConsumeBaselineObserved(output::Document&);
// Copies only an explicitly observed terminal result. No geometry is inferred.
sct::NonlinearSeparationResult CapturedNonlinearBaseline(
    const sct::CandidateFailureCapture&) noexcept;
prepared_replay::PairResult Evaluate(const nonlinear_fixture::Pair&, double duration,
    std::size_t work, unsigned depth, const sct::AcceptedEventCertificate*, std::size_t,
    bool observed_nonlinear_baseline = false);
bool Equivalent(const prepared_replay::PairResult&, const prepared_replay::PairResult&);
std::uint64_t ProfileHash(std::size_t work, unsigned depth,
                          std::size_t nonlinear_work, unsigned nonlinear_depth);
std::uint64_t DtHash(double duration, double kick_dt, std::uint64_t trajectory);
output::Document Geometry(const nonlinear_fixture::Pair&);
std::string JsonBytes(const output::Document&);
} // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test::failure_detail
