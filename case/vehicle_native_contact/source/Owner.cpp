#include "Internal.h"
#include "case/CanonicalWallArtifacts.h"
#include "case/vehicle_startup/physical_attachments/OriginalTiedPost.h"
#include <sstream>
namespace crash::cases::vehicle_native_contact::source::detail {
Owner PrepareOwner(const Inputs& in,const vehicle_run::OriginalPaths& paths,Limits limits,Forecast& forecast) {
    namespace wall=vehicle_wall::native;
    namespace start=vehicle_startup;
    const auto wall_budget=wall::WallSource::Preflight(in.domain,in.members.Input());
    Admit(forecast,forecast.owner_peak,0,wall_budget.peak_bytes,limits);
    const auto bytes=case_data::ReadPinnedWallManifest(paths.wall_manifest);
    case_data::CanonicalWall canonical_wall;std::istringstream stream(bytes);
    output::Require(canonical_wall.Load(stream).status==case_data::WallStatus::Ok,"Pinned native envelope wall manifest rejected");
    wall::Declaration declaration;declaration.profile=wall::Profile::EnvelopeFixedElasticV1;
    const auto made=wall::WallSource::Prepare(in.domain,in.members.Input(),canonical_wall,bytes,declaration);
    output::Require(made.report.status==wall::Status::Ready&&made.source,made.report.reason.c_str());
    Admit(forecast,forecast.owner_peak,made.source->forecast().peak_bytes,
        start::ForecastReferences(in.resolution,start::QephMetricProfile::AuthenticatedSourceLength,
            start::ReferenceLimits::CompleteRigidOverlay()).total_bytes,limits);
    const auto references=start::VehicleShellReferences::Prepare(in.resolution,
        start::QephMetricProfile::AuthenticatedSourceLength,start::ReferenceLimits::CompleteRigidOverlay());
    const auto mechanics_budget=wall::EnvelopePhysicalSource::PreflightWithControls(*made.source,references,in.packets);
    Admit(forecast,forecast.owner_peak,0,mechanics_budget.peak_bytes,limits);
    const auto mechanics=wall::EnvelopePhysicalSource::PrepareWithControls(*made.source,references,in.packets);
    const auto execution_budget=wall::EnvelopeExecutionSource::Preflight(mechanics);
    Admit(forecast,forecast.owner_peak,0,execution_budget.total_bytes,limits);
    const auto execution=wall::EnvelopeExecutionSource::Prepare(mechanics);
    // Reuse the qualified producer composition. Its bounded search may use CUDA;
    // the physical owner is still absent. Sum the published hard host bounds for
    // the small tied stages before entering that composition.
    const auto tied_bound=Add(Add(modelio::tied_shell::SearchGeometryLimits{}.host_bytes,
        start::TiedAssessmentLimits{}.host_bytes),Add(start::TiedFinalizationLimits{}.host_bytes,
        Add(modelio::tied_shell::AuxiliaryLimits{}.host_bytes,Add(modelio::tied_shell::ClassificationSourceLimits{}.host_bytes,
        Add(start::TiedClassificationLimits{}.host_bytes,start::TiedPostKinChkLimits{}.host_bytes)))));
    Admit(forecast,forecast.owner_peak,execution.forecast().total_bytes,tied_bound,limits);
    const auto& scope=in.domain.source();
    const auto finalized=start::physical_attachments::FinalizeOriginalTiedSearch(scope.tied_source(),in.members.vehicle);
    const auto auxiliary=modelio::tied_shell::TiedAuxiliaryConstraints::Prepare(scope.tied_source(),in.members.auxiliary,
        modelio::tied_shell::OriginalWallPolicy::ReplaceWithMeshWall);
    const auto context=modelio::tied_shell::TiedClassificationContext::Prepare(auxiliary,
        scope.point_mass_source().rigid_source(),in.members.wall,
        modelio::tied_shell::OriginalWallAssemblyPolicy::ReplaceWholeOriginalWallWithMeshWall);
    const auto post=start::physical_attachments::PrepareOriginalTiedPost(finalized,context);
    const auto joints=modelio::type45::VehicleType45Source::Prepare(in.domain,
        modelio::type45::Policy::OriginalDirectSdiType45NativeSupportsV6);
    const auto owner_budget=Owner::Preflight(execution,post,joints);
    Admit(forecast,forecast.owner_peak,0,owner_budget.peak_bytes,limits);
    return Owner::Prepare(execution,post,joints);
}
} // namespace crash::cases::vehicle_native_contact::source::detail
