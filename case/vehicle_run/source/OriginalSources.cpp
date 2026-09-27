#include "OriginalSources.h"
namespace crash::cases::vehicle_run::detail {
OriginalSources::OriginalSources(const OriginalPaths& paths,PhysicalProfile profile)
    :member(ReadOriginal(paths.member,42846753,"67208317e6c8eb1dd43b80001508915ccaace7bc0a745e1aa5a3b33f394df301")),
     canonical(ReadCanonical(paths,member)),
     plan(modelio::vehicle::VehicleSourcePlan::Read(canonical,paths.declarations,
        {3648589,"a96bc12b9c8467253da0898565c7875ad80f58f963b45d1dc405f5dddab76b1d"})),
     resolution(Resolve(plan,paths,member)),
     rigid(modelio::physical_scope::rigid::RigidPartSource::Prepare(plan,member)),
     masses(modelio::physical_scope::rigid::point_mass::Source::Prepare(rigid)),
     tied(modelio::tied_shell::TiedShellDeclaration::Prepare(canonical,member)),
     beams(modelio::type13::SourceType13::Read(paths.type13,
        {5150841,"c15fc2096317ac0206397ac50776f8456ddd23495e0c65aeee98e093ebd0b1b1"})),
     solids(modelio::solid_source::VehicleSolidSource::Prepare(canonical,member,
        SelectPhysical(profile).solids,
        SelectPhysical(profile).extended ? modelio::solid_source::Limits::ExtendedSolids()
            : modelio::solid_source::Limits{})) {
    if(SelectPhysical(profile).structural_beams)
        structural_beams.emplace(modelio::beam18::Source::Prepare(canonical,member,
            modelio::beam18::Policy::OriginalCircularFourPointLaw44V1));
}
} // namespace crash::cases::vehicle_run::detail
