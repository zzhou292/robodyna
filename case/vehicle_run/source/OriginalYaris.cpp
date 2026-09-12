#include "OriginalSources.h"
#include "case/CanonicalWallArtifacts.h"
#include "case/vehicle_startup/physical_attachments/VehiclePhysicalAttachments.h"
#include <sstream>
namespace crash::cases::vehicle_run {
namespace {
vehicle_startup::TiedSearchPostKinChk Classify(const detail::OriginalSources& source,const OriginalPaths& paths) {
    namespace tied=modelio::tied_shell;
    const auto geometry=tied::TiedShellSearchGeometry::Prepare(
        tied::TiedShellPacking::Prepare(source.tied),source.member);
    const auto finalized=vehicle_startup::TiedSearchFinalized::Prepare(
        vehicle_startup::TiedSearchAssessment::Prepare(geometry));
    const auto auxiliary=tied::TiedAuxiliaryConstraints::Prepare(source.tied,
        detail::ReadOriginal(paths.auxiliary_member,44991,
            "b93d5370a899f6f70299ea61cd55142c1f8b765b8ab7f9ac979d078486028929"),
        tied::OriginalWallPolicy::ReplaceWithMeshWall);
    const auto context=tied::TiedClassificationContext::Prepare(auxiliary,source.rigid,
        detail::ReadOriginal(paths.original_wall_member,10604,
            "ef02a4701b37d27cec81b1f9a02ab555f55ac61f68b070e8b0c18dc23b1d5155"),
        tied::OriginalWallAssemblyPolicy::ReplaceWholeOriginalWallWithMeshWall);
    return vehicle_startup::TiedSearchPostKinChk::Prepare(
        vehicle_startup::TiedSearchClassification::Prepare(finalized,context));
}
}
OriginalCase PrepareOriginalYaris(const OriginalPaths& paths,vehicle_wall::Settings settings,PhysicalProfile profile) {
    PhysicalProfileName(profile);
    vehicle_wall::CheckSettings(settings);
    const auto selected=detail::SelectPhysical(profile);
    detail::OriginalSources source(paths,profile);
    const auto scope=source.structural_beams
        ? modelio::physical_scope::PhysicalScope::PrepareVehicleSupports(
            source.masses,source.tied,source.beams,source.solids,*source.structural_beams)
        : modelio::physical_scope::PhysicalScope::Prepare(source.masses,source.tied,source.beams,source.solids);
    const auto domain=modelio::physical_domain::VehiclePhysicalDomain::Prepare(scope,selected.domain);
    const auto shells=vehicle_startup::VehicleShellBinding::Prepare(
        vehicle_startup::VehicleShellReferences::Prepare(source.resolution));
    const auto model_limits=selected.structural_beams ? vehicle_startup::physical_model::Limits::VehicleSupports()
        : selected.extended ? vehicle_startup::physical_model::Limits::ExtendedSolids()
        : vehicle_startup::physical_model::Limits{};
    const auto physical=vehicle_startup::physical_model::VehiclePhysicalModel::Prepare(domain,shells,model_limits);
    const auto post=Classify(source,paths);
    const auto attachments=vehicle_runtime::Attachments::Prepare(physical,post);
    const auto execution=vehicle_runtime::Execution::Prepare(physical);
    const auto joints=vehicle_startup::joints::VehicleJointModel::Prepare(physical,
        modelio::type45::VehicleType45Source::Prepare(domain,selected.joints));
    const auto bytes=case_data::ReadPinnedWallManifest(paths.wall_manifest);
    case_data::CanonicalWall wall;
    std::istringstream input(bytes);
    output::Require(wall.Load(input).status==case_data::WallStatus::Ok,"Pinned original wall manifest rejected");
    return {vehicle_wall::VehicleWallSetup::Prepare(execution,attachments,wall,bytes,settings),joints};
}
} // namespace crash::cases::vehicle_run
