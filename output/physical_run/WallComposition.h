#pragma once
#include "output/ArtifactIO.h"
#include <array>
#include <optional>
namespace crash::output::physical_run {
enum class CompositionProfile { RetainedV1, ExtendedSolidsV4, VehicleSupportsV5 };
// Immutable source/ledger observations, not an owner state or energy result.
// A legacy wall setup has no such receipt; absence must remain explicit.
struct WallComposition {
    CompositionProfile profile=CompositionProfile::RetainedV1;
    std::uint64_t physical_nodes=0,solid_parts=0,point_mass_records=0;
    // LAW36 solid18, HEPH24, S6Z, LAW44 solid18, LAW90 solid18.
    std::array<std::uint64_t,5> solid_parents{};
    std::uint64_t part_roots=0,rigid_groups=0,rigid_members=0;
    std::uint64_t plain_complete=0,plain_restricted=0,plain_omitted=0;
    double initial_mass_kg=0,point_mass_kg=0;
    std::uint64_t structural_beam_parents=0,structural_beam_parts=0;
};
Document WallCompositionDocument(const WallComposition&);
WallComposition ReadWallComposition(const Value&);
void CheckWallBeamObservation(bool beam18,const std::optional<WallComposition>&);
// Checks the enclosing version and actual source/count bounds. V1 is supported
// without inferring a composition; V2 requires its complete typed receipt.
std::optional<WallComposition> ReadSetupComposition(const Value& setup,
    std::uint64_t surface_nodes,std::uint64_t canonical_nodes);
} // namespace crash::output::physical_run
