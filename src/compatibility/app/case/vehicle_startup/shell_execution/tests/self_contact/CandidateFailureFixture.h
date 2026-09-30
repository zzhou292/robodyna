#pragma once

#include "NonlinearCoverageFixture.h"
#include "PreparedCensusReplay.h"
#include "lib_src/collision/self_contact_transaction/CandidateFailureCapture.h"
#include "output/ArtifactIO.h"

#include <array>
#include <exception>
#include <filesystem>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test {

namespace contact = tlfea::contact;
namespace sct = contact::self_contact_transaction;

inline constexpr std::size_t FailureCaptureHostCap = 8u << 20;
inline constexpr std::size_t FailureFixtureArchiveCap = 2u << 20;
inline constexpr std::size_t FailureFixtureManifestCap = 128u << 10;

// Qualification only. No live authority or borrowed geometry survives callback
// return. Errors are diagnostic; they cannot turn native rejection into success.
class CandidateFailureFixture {
  public:
    explicit CandidateFailureFixture(const contact::SelfContactTransactionLimits&);
    CandidateFailureFixture(const CandidateFailureFixture&) = delete;
    CandidateFailureFixture& operator=(const CandidateFailureFixture&) = delete;
    sct::CandidateFailureObserver observer() noexcept;
    bool seen() const noexcept { return seen_; }
    bool complete() const noexcept { return complete_; }
    bool owners_equivalent() const noexcept { return owners_equivalent_; }
    const char* error() const noexcept { return error_.data(); }
    const contact::SelfContactTransactionReport& report() const noexcept { return report_; }
    const nonlinear_fixture::PhaseIdentity& phase() const noexcept { return phase_; }
    const nonlinear_fixture::Pair& pair() const;
    std::string Export(const std::filesystem::path& absent_directory) const;
  private:
    static void Capture(void*, const sct::CandidateFailureCapture&) noexcept;
    void Freeze(const sct::CandidateFailureCapture&);
    std::size_t work_ = 0;
    unsigned depth_ = 0;
    std::size_t nonlinear_work_ = 0;
    unsigned nonlinear_depth_ = 0;
    double duration_ = 0, kick_dt_ = 0;
    std::uint64_t owner_ = 0, attempt_ = 0;
    bool seen_ = false, complete_ = false, owners_equivalent_ = false;
    std::array<char, 512> message_{};
    std::array<char, 256> error_{};
    std::exception_ptr exception_;
    contact::SelfContactTransactionReport report_;
    nonlinear_fixture::PhaseIdentity phase_;
    sct::QualificationPreparedActivitySummary activity_;
    sct::LinearResidualSeparationResult residual_;
    prepared_replay::PairResult compact_, full_;
    std::size_t full_owners_ = 0;
    std::vector<nonlinear_fixture::Pair> pairs_;
};

// Bounded host-only classification of one caller-hashed failure artifact.
// The returned policy is diagnostic and never authorizes a physical step.
output::Document ReplayCandidateFailure(const std::filesystem::path& manifest,
                                       const std::string& expected_sha256);

} // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
