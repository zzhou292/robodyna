#pragma once
#include <array>
#include <cstdint>

namespace crash::cases::source_part_elastic {
// Accepted case ledger; contact result provenance stays PreparedCandidate and
// is interpreted alongside the actual accepted owner stamp. At epoch zero the
// immutable setup certifies separation and there is no candidate result.
struct WallMetrics {
    double wall_kick_impulse=0,wall_kick_impulse_error=0;
    std::array<double,3> carried_momentum_residual{},carried_momentum_allowance{};
    // Orbital x uses accepted endpoint time; carried v/omega retain the raw
    // velocity time in Snapshot::stamp. These angular fields are observations,
    // not a separately qualified angular-momentum acceptance gate.
    std::array<double,3> carried_angular_momentum{},wall_kick_moment{},wall_kick_moment_error{};
    double synchronized_kinetic_uncertainty=0,physical_energy_uncertainty=0,energy_allowance=0;
    std::uint64_t first_contact_epoch=0,last_contact_epoch=0,contact_intervals=0;
    std::uint32_t active_nodes=0;
};
} // namespace crash::cases::source_part_elastic
