#include "Internal.h"
#include "case/vehicle_startup/joints/VehicleJointModel.h"
#include "case/vehicle_self_contact/native/PostGapmMainSource.h"
namespace crash::cases::vehicle_native_contact::source::detail {
Contact PrepareContact(const Inputs& in,const Owner& owner,Limits limits,Forecast& forecast) {
    namespace start=vehicle_startup;
    namespace contact=vehicle_self_contact::native;
    using Physical=start::physical_model::VehiclePhysicalModel;
    const auto held=owner.retained_host_upper_bound(limits.host_bytes);
    const auto admit=[&](std::size_t bytes){Admit(forecast,forecast.contact_peak,held,bytes,limits);};
    // The contact pre-correction branch preserves the original shell reference
    // metric; it is separate from the envelope's AuthenticatedSourceLength graph.
    admit(start::ForecastReferences(in.resolution,start::ReferenceLimits::CompleteRigidOverlay()).total_bytes);
    const auto references=start::VehicleShellReferences::Prepare(in.resolution,start::ReferenceLimits::CompleteRigidOverlay());
    admit(start::ForecastShellBinding(references).total_bytes);
    const auto shells=start::VehicleShellBinding::Prepare(references);
    const auto model_limits=start::physical_model::Limits::VehicleSupports();
    admit(Physical::PreflightWithControls(in.domain,shells,in.packets,model_limits).total_bytes);
    const auto physical=Physical::PrepareWithControls(in.domain,shells,in.packets,model_limits);
    const auto joints=start::joints::VehicleJointModel::Prepare(physical,owner.joint_source());
    admit(contact::nodal_seed::PreCorrectionNodalSource::Preflight(physical,joints,in.members.Input()).peak_bytes);
    const auto before=contact::nodal_seed::PreCorrectionNodalSource::Prepare(physical,joints,in.members.Input());
    admit(contact::nodal_correction::CorrectedNodalSource::Preflight(before,in.members.Input()).peak_bytes);
    const auto corrected=contact::nodal_correction::CorrectedNodalSource::Prepare(before,in.members.Input());
    output::Require(corrected.report.status==contact::nodal_correction::Status::Ready&&corrected.source,corrected.report.reason.c_str());
    const auto selection=modelio::self_contact::OriginalSelection::Prepare(in.canonical,in.members.auxiliary,in.members.combine);
    admit(contact::initial_surfaces::InitialSurfaceSource::Preflight(*corrected.source,selection).peak_bytes);
    const auto initial=contact::initial_surfaces::InitialSurfaceSource::Prepare(*corrected.source,selection,in.members.vehicle);
    output::Require(initial.report.status==contact::initial_surfaces::Status::Ready&&initial.source,initial.report.reason.c_str());
    admit(contact::mixed_interface::MixedInterfaceSource::Preflight(*initial.source).peak_bytes);
    const auto mixed=contact::mixed_interface::MixedInterfaceSource::Prepare(*initial.source);
    output::Require(mixed.report.status==contact::mixed_interface::Status::Ready&&mixed.source,mixed.report.reason.c_str());
    // The gap producer retains corrected context, while the mixed surface
    // graph already exists. Charge their published bounds without subtracting
    // unexposed internal shared storage.
    admit(Add(mixed.source->retained_host_upper_bound(limits.host_bytes),
        contact::gap_operands::ContactGapOperands::Preflight(*corrected.source).peak_bytes));
    const auto gaps=contact::gap_operands::ContactGapOperands::Prepare(*corrected.source);
    output::Require(gaps.report.status==contact::gap_operands::Status::Ready&&gaps.source,gaps.report.reason.c_str());
    admit(contact::post_gapm::PostGapmMainSource::Preflight(*mixed.source,*gaps.source).peak_bytes);
    const auto main=contact::post_gapm::PostGapmMainSource::Prepare(*mixed.source,*gaps.source,in.members.combine);
    output::Require(main.report.status==contact::post_gapm::Status::Ready&&main.source,main.report.reason.c_str());
    const vehicle_wall::native::wall_interface::Declaration declaration{
        vehicle_wall::native::wall_interface::Profile::AllRetainedVehicleNodesToFixedMeshV1,1,1};
    Admit(forecast,forecast.contact_peak,0,
        Wall::Preflight(owner,*main.source,declaration).complete_construction_bound,limits);
    const auto wall=Wall::Prepare(owner,*main.source,declaration);
    output::Require(wall.report.status==vehicle_wall::native::wall_interface::Status::Prepared&&wall.source,wall.report.reason.c_str());
    Admit(forecast,forecast.contact_peak,0,wall.source->forecast().complete_construction_bound,limits);
    const auto& embedding=owner.execution_source().mechanical().embedding();
    admit(Add(wall.source->forecast().own_retained,Self::Preflight(*main.source,embedding).peak_bytes));
    const auto self=Self::Prepare(*main.source,embedding);
    output::Require(self.report.status==contact::mixed_starter::Status::Ready&&self.source,self.report.reason.c_str());
    const auto& wall_input=owner.execution_source().mechanical().wall();
    output::Require(self.source->forecast().retained_bytes>=main.source->forecast().retained_bytes,
        "Native self source retained partition is inconsistent");
    const auto self_extra=self.source->forecast().retained_bytes-main.source->forecast().retained_bytes;
    admit(Add(Add(wall.source->forecast().own_retained,self_extra),
        Controls::Preflight(*main.source,in.members.Input(),&wall_input).peak_bytes));
    const auto controls=Controls::Prepare(*main.source,in.members.Input(),&wall_input);
    output::Require(controls.report.status==contact::initial_controls::Status::Ready&&controls.source,controls.report.reason.c_str());
    return {*self.source,*wall.source,*controls.source};
}
} // namespace crash::cases::vehicle_native_contact::source::detail
