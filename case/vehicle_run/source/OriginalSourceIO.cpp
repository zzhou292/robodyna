#include "OriginalSourceIO.h"
#include "modelio/vehicle_sections/VehicleSectionResolution.h"
namespace crash::cases::vehicle_run::detail {
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
std::string ReadOriginal(const std::filesystem::path& path,std::size_t bytes,const char* sha256) {
    auto result=output::ReadBounded(path,bytes);
    output::Require(result.size()==bytes && output::Sha256(result)==sha256,
        "Pinned original source member or declaration identity differs");
    return result;
}
} // namespace crash::cases::vehicle_run::detail
