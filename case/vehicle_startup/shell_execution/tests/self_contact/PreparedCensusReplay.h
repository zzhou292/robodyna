#pragma once

#include "NonlinearCoverageFixture.h"
#include "output/ArtifactIO.h"

#include <array>
#include <filesystem>
#include <functional>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test::prepared_replay {

inline constexpr std::size_t ArchiveByteCap = std::size_t{2} << 30;
inline constexpr std::size_t ShardByteCap = 64u << 20;
inline constexpr std::size_t ManifestByteCap = 1u << 20;
inline constexpr std::size_t PairsPerShard = 128;

struct PairResult {
    std::string file, family;
    std::size_t ordinal = 0;
    tlfea::contact::FixedTriangleKey facets[2];
    tlfea::contact::self_contact_transaction::NonlinearSeparationStatus baseline_status{};
    std::size_t baseline_work = 0, owners = 0, exclusions = 0;
    unsigned baseline_depth = 0;
    bool affine = false;
    // In-memory diagnostic provenance only; no binary fixture/schema field.
    // Failure manifest v2 carries explicit baseline_observed; legacy v1 is false.
    bool observed_failure_baseline = false;
    // The certificate ordinal addresses the frozen subset; source_order is the
    // retained live ledger identity. Neither result authorizes a physical step.
    tlfea::contact::self_contact_transaction::NonlinearSeparationResult ledger, policy;
};

struct Summary {
    std::size_t files = 0, bytes = 0, pairs = 0, linear = 0, nonlinear = 0;
    std::size_t noncertified = 0, total_work = 0;
    std::size_t omitted_nonlinear_persistent = 0, omitted_linear_persistent = 0;
    std::array<std::size_t, static_cast<unsigned>(
        tlfea::contact::self_contact_transaction::NonlinearSeparationStatus::CertifiedLocalIntersection) + 1> status_counts{};
    std::uint64_t result_digest = 1469598103934665603ull;
    bool complete = false;
};

bool Certified(tlfea::contact::self_contact_transaction::NonlinearSeparationStatus) noexcept;
const char* StatusName(tlfea::contact::self_contact_transaction::NonlinearSeparationStatus) noexcept;
std::vector<tlfea::contact::self_contact_transaction::AcceptedFeatureExclusionCertificate>
SameRigidExclusions(const nonlinear_fixture::Pair&);

// Verifies the caller-pinned manifest and every shard. Holds one decoded shard
// at a time and calls observe once for EVERY pair, including certified pairs.
// Throws on invalid evidence; no complete result is returned after a failure.
Summary Replay(const std::filesystem::path& manifest, const std::string& expected_sha256,
               const std::function<void(const PairResult&)>& observe);
output::Document PairDocument(const PairResult&);
output::Document SummaryDocument(const Summary&, const std::string& manifest_sha256);

}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test::prepared_replay
