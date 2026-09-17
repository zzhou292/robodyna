#pragma once

#include "PreparedCensusReplay.h"

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test::prepared_replay::detail {

struct Shard {
    std::string file, sha256, family;
    std::uint64_t source_hash = 0, payload_hash = 0, roster_digest = 0;
    std::size_t bytes = 0, pairs = 0;
};
struct Manifest {
    std::vector<Shard> shards;
    std::size_t bytes = 0, fixture_bytes = 0, linear = 0, nonlinear = 0, work = 0;
    unsigned depth = 0;
    std::uint64_t profile_hash = 0, dt_hash = 0, census_digest = 0, epoch = 0;
    double accepted_time = 0, prepared_time = 0, duration = 0, kick_dt = 0;
};

Manifest ReadManifest(const std::filesystem::path&, const std::string& expected_sha256);
nonlinear_fixture::File ReadShard(const std::filesystem::path&, const Shard&, const Manifest&);

}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test::prepared_replay::detail
