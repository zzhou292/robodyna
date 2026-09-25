#include "OriginalSources.h"
namespace crash::cases::vehicle_run::detail {
namespace {
namespace records=output::full_shell;
records::source::CanonicalSource ReadCanonical(const OriginalPaths& paths,const std::string& member) {
    records::source::SourceInputs input;
    input.canonical_root=paths.canonical;
    input.scope_root=paths.scope.parent_path();
    input.member_root=paths.member.parent_path();
    input.canonical_manifest={"manifest.json",
        "c82f1886b8935d69ff7db4c29c700370e3a057579fab80d02664a253bc7af1c8",2632394};
    input.scope_report={paths.scope.filename().string(),
        "fdb51869dfd3f4de265bb4494a2d0f904c5c466bf9962b71ce19e9098a761ff0",13212691};
    input.source_member={paths.member.filename().string(),
        "67208317e6c8eb1dd43b80001508915ccaace7bc0a745e1aa5a3b33f394df301",42846753};
    input.tire_policy="omit_original_tire_shells";
    input.units={"t","mm","s",1000,.001,1};
    return records::source::CanonicalSource::ReadWithMemberBytes(input,member);
}
modelio::vehicle::VehicleSectionResolution Resolve(const modelio::vehicle::VehicleSourcePlan& plan,
    const OriginalPaths& paths,const std::string& member) {
    using Resolution=modelio::vehicle::VehicleSectionResolution;
    using Profile=modelio::vehicle::ResolutionProfile;
    const auto bytes=ReadOriginal(paths.glass_resolution,190529,
        "ea40b817b66c73e961c502e5ff0d4dffd5d354b65e1339e6a093164adeb32158");
    const auto glass=Resolution::ReadBytes(plan,bytes,{bytes.size(),output::Sha256(bytes)});
    const auto midlayer=Resolution::ResolveOriginalMidlayer(glass,Profile::OriginalMidlayerV1);
    return Resolution::ResolveOriginalRigidParts(midlayer, member, Profile::OriginalRigidPartsV1,
        modelio::vehicle::ResolutionLimits::CompleteRigidOverlay());
}
}
std::string ReadOriginal(const std::filesystem::path& path,std::size_t bytes,const char* sha256) {
    auto result=output::ReadBounded(path,bytes);
    output::Require(result.size()==bytes && output::Sha256(result)==sha256,
        "Pinned original source member or declaration identity differs");
    return result;
}
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
