#pragma once

#include "NonlinearCoverageFixture.h"

namespace crash::cases::vehicle_startup::shell_execution::
    self_contact_test::linear_fixture {

namespace storage = nonlinear_fixture;
using Pair = storage::Pair;
using File = storage::File;
using PhaseIdentity = storage::PhaseIdentity;

// Filled from the sole authenticated source census, then enforced by the
// seconds-scale fixture replay.
inline constexpr std::size_t ExpectedPairs = 1;
inline constexpr std::uint64_t ExpectedRosterDigest =
    10312651066629367413ull;
inline constexpr std::uint64_t ExpectedCensusDigest =
    6473154677596308446ull;
inline constexpr std::uint64_t ExpectedSchemaHash =
    1586894878486140084ull;
inline constexpr std::uint64_t ExpectedSourceHash =
    8357837849776735536ull;
inline constexpr std::uint64_t ExpectedProfileHash =
    5356721435267868170ull;
inline constexpr std::uint64_t ExpectedDtHash =
    16186267176476366842ull;
inline constexpr std::uint64_t ExpectedPayloadHash =
    12401461440924532527ull;
inline constexpr std::uint64_t ExpectedPayloadBytes = 21200;
inline constexpr std::uint64_t ExpectedPolicyResultDigest =
    12671290143471704819ull;

inline std::uint64_t RosterDigest(
    const std::vector<Pair>& pairs) noexcept {
    return storage::RosterDigest(pairs);
}

inline std::uint64_t SourceHash(
    const std::vector<Pair>& pairs) noexcept {
    return storage::SourceHash(pairs);
}

inline void Write(
    const std::string& path, const std::vector<Pair>& pairs,
    std::uint64_t profile_hash, std::uint64_t dt_hash,
    std::uint64_t census_digest, const PhaseIdentity& phase) {
    storage::Write(
        path, pairs, profile_hash, dt_hash, census_digest,
        phase, false);
}

inline File Read(const std::string& path) {
    const auto result = storage::Read(path, false);
    if ((ExpectedPairs && result.pairs.size() != ExpectedPairs) ||
        (ExpectedRosterDigest &&
         result.roster_digest != ExpectedRosterDigest) ||
        (ExpectedCensusDigest &&
         result.nonlinear_roster_digest !=
             ExpectedCensusDigest) ||
        (ExpectedSchemaHash &&
         result.schema_hash != ExpectedSchemaHash) ||
        (ExpectedSourceHash &&
         result.source_hash != ExpectedSourceHash) ||
        (ExpectedProfileHash &&
         result.profile_hash != ExpectedProfileHash) ||
        (ExpectedDtHash &&
         result.dt_hash != ExpectedDtHash) ||
        (ExpectedPayloadHash &&
         result.payload_hash != ExpectedPayloadHash) ||
        (ExpectedPayloadBytes &&
         result.payload_bytes != ExpectedPayloadBytes))
        throw std::runtime_error(
            "Linear fixture digest or count changed");
    return result;
}

}  // namespace crash::cases::vehicle_startup::shell_execution::
   // self_contact_test::linear_fixture
