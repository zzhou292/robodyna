#pragma once
#include "../ContactNodalSeed.h"
#include "modelio/native_spring_ids/Resolve.h"
#include "lib_src/collision/RadiossType25Coefficients.h"
#include "lib_src/collision/RadiossType25NodalSeed.h"
#include "lib_src/collision/RadiossType25ShellSource.h"
#include "output/ArtifactIO.h"

namespace crash::cases::vehicle_self_contact::native::nodal_seed::detail {
namespace ids = modelio::native_spring_ids;
namespace shells = n::source_shells;
using output::Require;

struct Packed {
    std::vector<Contributor> contributors;
    std::vector<n::NativeVolumeOccurrence> volumes;
    std::vector<n::NativeStiffnessOccurrence> stiffness;
    std::vector<shells::PhysicalShell> shells;
};

Counts CheckModel(const PhysicalModel&, const JointModel&, Limits);
Forecast Budget(const PhysicalModel&, const JointModel&, const ids::ImportMembers&, Limits);
struct InteriorPrepared { modelio::solid_control::DirectSource declarations; InteriorDisposition disposition; };
InteriorPrepared ReadInterior(const PhysicalModel&, const ids::ImportMembers&, Limits);
void PackSolids(const PhysicalModel&, n::UnitScale, Packed&);
void PackDirect(const PhysicalModel&, const JointModel&, const ids::Resolution&, n::UnitScale, Packed&);
void PackShells(const PhysicalModel&, n::UnitScale, Packed&);
std::string ContributorDigest(const Packed&, const Provenance&, std::size_t byte_cap);

inline n::coefficient_detail::UnitFactors Factors(n::UnitScale units) {
    n::coefficient_detail::UnitFactors result;
    Require(n::coefficient_detail::Make(units, result), "Invalid native contact seed working units");
    return result;
}
inline bool SameUnits(n::UnitScale a, n::UnitScale b) {
    return output::Bits(a.length_m) == output::Bits(b.length_m) &&
        output::Bits(a.mass_kg) == output::Bits(b.mass_kg) &&
        output::Bits(a.time_s) == output::Bits(b.time_s);
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_seed::detail
