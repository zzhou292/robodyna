#pragma once

#include "CandidateCapture.h"
#include "output/ArtifactIO.h"

#include <filesystem>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test {

struct CensusFixtureExportResult {
    std::size_t linear_pairs = 0;
    std::size_t nonlinear_pairs = 0;
    std::size_t files = 0;
    std::size_t bytes = 0;
};

// Writes the complete affine WorkExhausted roster and every remaining nonlinear
// geometry/owner case. Earlier certified classes are counted and hashed in the
// manifest; no source-ID whitelist selects the new fixture.
CensusFixtureExportResult ExportPreparedCensus(
    const vehicle_self_contact::AcceptedAssemblyCouponSnapshot&,
    const std::filesystem::path& destination,
    const output::Document& profile, std::uint64_t profile_hash,
    std::uint64_t dt_hash);

}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
