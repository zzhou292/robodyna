#include "GuidedPlateCaseSupport.h"
#include "lib_src/solvers/ExplicitNodalStep.h"
#include <algorithm>
#include <cmath>
#include <exception>

namespace crash::case_data {
namespace detail=guided_detail;
namespace fea=tl::fea;
namespace shell=tl::fea::reissner;
namespace contact=tlfea::contact;
namespace ref=crash::reference;
using Code=GuidedPlateStatus;

struct GuidedPlateCase::Impl {
    detail::ContactWall wall;
    std::unique_ptr<ref::GuidedPlateModel> model;
    ref::GuidedPlateModalReport modal;
    fea::FENodalState state;
    shell::ReissnerShellBatch elements;
    contact::Q4PlanarContact contact;
    visual::NodalMeshOutput output;
    GuidedPlateMetrics metrics;
    shell::ShellBatchDiagnostics element_association;
    contact::Q4PlanarContactDiagnostics contact_association;
    ref::ElasticCouponConfiguration audit_configuration;
    std::uint64_t audit_stride=1;
    double expected_h=0;
    bool usable=true;

    GuidedPlateReport State(const fea::NodalReport& r) {
        if (r.status==fea::NodalStatus::DeviceFailure) usable=false;
        return {Code::StateFailure,r.message};
    }
    GuidedPlateReport Element(const shell::ShellBatchReport& r) {
        if (r.status==shell::ShellBatchStatus::kDeviceFailure) usable=false;
        return {Code::ElementFailure,detail::Describe(r)};
    }
    GuidedPlateReport Contact(const contact::Q4PlanarContactReport& r) {
        if (r.status==contact::Q4PlanarContactStatus::DeviceFailure) usable=false;
        return {Code::ContactFailure,detail::Describe(r)};
    }
    GuidedPlateReport CopyResults(GuidedPlateFrame& frame) {
        auto e=elements.CopyElementResults(element_association,frame.element.data(),frame.element.size());
        contact::Q4PlanarContactReport c;
        if (e.status==shell::ShellBatchStatus::kSuccess)
            c=contact.CopyParentResults(contact_association,frame.parent.data(),frame.parent.size());
        const bool stale=e.status==shell::ShellBatchStatus::kStaleTrial ||
            (e.status==shell::ShellBatchStatus::kSuccess && c.status==contact::Q4PlanarContactStatus::StaleAttempt);
        if (stale) {
            detail::TrialScope scope{state}; fea::NodalTrialToken token; fea::NodalAssemblyView assembly;
            const auto begin=state.BeginTrial(&token,&assembly);
            if (begin.status!=fea::NodalStatus::Ok) return State(begin);
            shell::ShellBatchDiagnostics refreshed_shell;
            contact::Q4PlanarContactDiagnostics refreshed_contact;
            e=elements.Assemble(assembly,&refreshed_shell);
            if (e.status!=shell::ShellBatchStatus::kSuccess) return Element(e);
            c=contact.Assemble(assembly,&refreshed_contact);
            if (c.status!=contact::Q4PlanarContactStatus::Ok) return Contact(c);
            if (!detail::Matches(refreshed_shell,refreshed_contact,assembly.owner_id,
                                 assembly.accepted.base_epoch,assembly.attempt,false))
                return {Code::OutputFailure,"Refreshed guided results have mismatched accepted identities"};
            e=elements.CopyElementResults(refreshed_shell,frame.element.data(),frame.element.size());
            if (e.status!=shell::ShellBatchStatus::kSuccess) return Element(e);
            c=contact.CopyParentResults(refreshed_contact,frame.parent.data(),frame.parent.size());
            if (c.status!=contact::Q4PlanarContactStatus::Ok) return Contact(c);
            element_association=refreshed_shell; contact_association=refreshed_contact;
        } else {
            if (e.status!=shell::ShellBatchStatus::kSuccess) return Element(e);
            if (c.status!=contact::Q4PlanarContactStatus::Ok) return Contact(c);
        }
        if (!detail::MatchesAcceptedResults(element_association,contact_association,metrics.stamp))
            return {Code::OutputFailure,"Guided result scratch does not describe the accepted endpoint"};
        frame.element_association=element_association; frame.contact_association=contact_association;
        return {Code::Ok,"Guided contributor results staged"};
    }
};

GuidedPlateCase::GuidedPlateCase()=default;
GuidedPlateCase::~GuidedPlateCase()=default;
GuidedPlateReport GuidedPlateCase::Initialize(const CanonicalWall& wall,const GuidedPlateConfig& config) {
    if (impl_) return {Code::AlreadyInitialized,"Guided plate is already initialized"};
    if ((config.refinement!=1 && config.refinement!=2 && config.refinement!=4) ||
        !config.diagnostic_intervals || config.diagnostic_intervals>64)
        return {Code::InvalidInput,"Invalid guided fixed-step refinement or diagnostic cadence"};
    try {
        auto s=std::make_unique<Impl>(); std::string error;
        if (!detail::CopyCanonicalWall(wall,s->wall,error)) return {Code::InvalidInput,error};
        s->model=std::make_unique<ref::GuidedPlateModel>(s->wall.view(),kGuidedPlateWallBinding);
        if (ref::AuditGuidedPlate(*s->model,s->modal,error)!=ref::ElasticCouponStatus::kSuccess)
            return {Code::AuditFailure,error};
        if (!s->modal.step_count || s->modal.step_count>100000)
            return {Code::AdmissionFailure,"Guided horizon exceeds the initial base-step budget"};
        const auto& data=s->model->shell().data(); const auto& guided=s->model->data();
        detail::InitialState initial(s->modal.initial_configuration);
        fea::NodalStateConfig state_config; state_config.node_count=ref::kCouponNodes;
        s->expected_h=state_config.fixed_dt=s->modal.time_step/config.refinement;
        auto state=s->state.Initialize(state_config,{initial.x.data(),initial.v.data(),initial.omega.data(),ref::kCouponNodes,initial.q.data()},
            data.inverse_mass.data(),{guided.translation_fixed_bits.data(),guided.rotation_fixed.data(),data.inverse_isotropic_inertia.data()});
        if (state.status!=fea::NodalStatus::Ok) return s->State(state);
        const auto elements=detail::ShellElements(data);
        shell::ReissnerShellBatchConfig element_config;
        element_config.owner=s->state.accepted(); element_config.configuration_id=kGuidedPlateQualification;
        element_config.element_count=ref::kCouponElements;
        element_config.drilling_policy=shell::ShellDrillingInertiaPolicy::kEqualPhysicalTangential;
        auto element=s->elements.Initialize(element_config,elements.data());
        if (element.status!=shell::ShellBatchStatus::kSuccess) return s->Element(element);
        contact::Q4PlanarContactConfig contact_config;
        contact_config.owner=s->state.accepted(); contact_config.configuration_id=kGuidedPlateQualification;
        contact_config.wall_binding_id=guided.wall_binding_id; contact_config.stiffness_per_area=guided.stiffness_per_area;
        contact_config.maximum_penetration=guided.maximum_penetration; contact_config.exposed_clearance=guided.exposed_clearance;
        contact_config.integration=guided.integration;
        auto collision=s->contact.Initialize(contact_config,s->wall.view(),
            {{initial.x.data(),ref::kCouponNodes,3,1},{initial.v.data(),ref::kCouponNodes,3,1},guided.parents.data(),ref::kCouponElements},
            {data.inverse_mass.data(),guided.translation_fixed_bits.data(),ref::kCouponNodes,0});
        if (collision.status!=contact::Q4PlanarContactStatus::Ok) return s->Contact(collision);
        if (s->contact.stiffness_rate_bound()!=s->modal.contact_rate_bound)
            return {Code::AdmissionFailure,"Runtime contact rate differs from the immutable host audit"};
        if (s->state.allocations().device_bytes+s->elements.allocations().device_bytes+s->contact.allocations().device_bytes>
            kGuidedPlateDeviceBudget)
            return {Code::AdmissionFailure,"Guided modules exceed the one-MiB explicit device budget"};
        {
            detail::TrialScope scope{s->state}; fea::NodalTrialToken token; fea::NodalAssemblyView assembly;
            state=s->state.BeginTrial(&token,&assembly);
            if (state.status!=fea::NodalStatus::Ok) return s->State(state);
            element=s->elements.Assemble(assembly,&s->metrics.shell);
            if (element.status!=shell::ShellBatchStatus::kSuccess) return s->Element(element);
            collision=s->contact.Assemble(assembly,&s->metrics.contact);
            if (collision.status!=contact::Q4PlanarContactStatus::Ok) return s->Contact(collision);
            if (!detail::Matches(s->metrics.shell,s->metrics.contact,assembly.owner_id,assembly.accepted.base_epoch,assembly.attempt,false))
                return {Code::AdmissionFailure,"Initial guided contributor association mismatch"};
        }
        s->metrics.initial_energy=ShellKineticEnergy(s->metrics.shell)+s->metrics.shell.elastic_energy+s->metrics.contact.potential.value;
        if (!CheckGuidedPlateEnvelope(s->metrics.shell,s->metrics.contact,s->modal,s->metrics.initial_energy,
                                      ElasticShellLimits::displacement,false,s->metrics.work,error))
            return {Code::AdmissionFailure,error};
        if (s->metrics.work.kinetic_energy!=0 ||
            std::abs(s->metrics.shell.elastic_energy-s->modal.initial_elastic_energy)>1e-8*s->metrics.initial_energy)
            return {Code::AuditFailure,"Initial guided CPU/CUDA energy or zero-velocity contract disagrees"};
        s->element_association=s->metrics.shell; s->contact_association=s->metrics.contact;
        s->metrics.stamp=s->state.accepted(); s->metrics.required_steps=s->modal.step_count*config.refinement;
        s->metrics.last_operator_norm=s->modal.sampled_structural_operator_norm[4];
        s->audit_stride=((s->modal.step_count+config.diagnostic_intervals-1)/config.diagnostic_intervals)*config.refinement;
        auto published=s->output.Initialize(s->state,detail::SurfaceBinding(s->metrics.stamp,guided));
        if (published.status!=visual::Status::Ok) return {Code::OutputFailure,published.message};
        published=s->output.Publish(s->state);
        if (published.status!=visual::Status::Ok) return {Code::OutputFailure,published.message};
        GuidedPlateReport success{Code::Ok,"Guided plate initialized"};
        impl_=std::move(s); return success;
    } catch (const std::exception& error) { return {Code::InvalidInput,error.what()}; }
}

GuidedPlateReport GuidedPlateCase::Step(const GuidedPlateStepRequest& request) {
    if (!impl_) return {Code::NotInitialized,"Guided plate is not initialized"};
    auto& s=*impl_;
    if (!s.usable) return {Code::StateFailure,"Guided plate stopped after a CUDA failure"};
    if (!std::isfinite(request.maximum_displacement) || request.maximum_displacement<=0 ||
        request.maximum_displacement>ElasticShellLimits::displacement || s.metrics.stamp.epoch>=s.metrics.required_steps)
        return {Code::InvalidInput,"Invalid guided stop envelope or completed admitted horizon"};
    if (!detail::SameStamp(s.metrics.stamp,s.state.accepted()) || s.metrics.stamp.fixed_dt!=s.expected_h)
        return {Code::StateFailure,"Guided accepted owner metadata changed outside its coordinator"};
    detail::TrialScope scope{s.state}; fea::NodalTrialToken token; fea::NodalAssemblyView assembly;
    auto state=s.state.BeginTrial(&token,&assembly);
    if (state.status!=fea::NodalStatus::Ok) return s.State(state);
    shell::ShellBatchDiagnostics base_shell,candidate_shell;
    contact::Q4PlanarContactDiagnostics base_contact,candidate_contact;
    auto element=s.elements.Assemble(assembly,&base_shell);
    if (element.status!=shell::ShellBatchStatus::kSuccess) return s.Element(element);
    auto collision=s.contact.Assemble(assembly,&base_contact);
    if (collision.status!=contact::Q4PlanarContactStatus::Ok) return s.Contact(collision);
    if (!detail::Matches(base_shell,base_contact,assembly.owner_id,assembly.accepted.base_epoch,assembly.attempt,false))
        return {Code::AdmissionFailure,"Guided base contributor association mismatch"};
    state=s.state.SealAssembly(token); if (state.status!=fea::NodalStatus::Ok) return s.State(state);
    const fea::NodalStepAdmission admission{assembly.owner_id,assembly.accepted.base_epoch,assembly.attempt,
        s.modal.proposed_step_limit,ElasticShellLimits::rotation_increment,fea::NodalStepAdmissionKind::RestrictedElasticTrajectory,
        kGuidedPlateQualification,s.modal.combined_rate_envelope};
    state=fea::AdvanceNodal(s.state,token,admission); if (state.status!=fea::NodalStatus::Ok) return s.State(state);
    fea::NodalPreparedView prepared;
    state=s.state.BorrowPrepared(token,&prepared); if (state.status!=fea::NodalStatus::Ok) return s.State(state);
    if (!detail::MatchesPrepared(prepared,assembly,s.metrics.stamp))
        return {Code::AdmissionFailure,"Guided prepared owner/token/consumed-step association mismatch"};
    element=s.elements.EvaluateCandidate(prepared,&candidate_shell);
    if (element.status!=shell::ShellBatchStatus::kSuccess) return s.Element(element);
    collision=s.contact.EvaluateCandidate(prepared,&candidate_contact);
    if (collision.status!=contact::Q4PlanarContactStatus::Ok) return s.Contact(collision);
    if (!detail::Matches(candidate_shell,candidate_contact,prepared.owner_id,prepared.kinematics.base_epoch,prepared.attempt,true))
        return {Code::AdmissionFailure,"Guided candidate contributor association mismatch"};
    auto next=s.metrics; std::string error;
    if (!CheckGuidedPlateEnvelope(candidate_shell,candidate_contact,s.modal,s.metrics.initial_energy,
                                  request.maximum_displacement,true,next.work,error))
        return {Code::AdmissionFailure,error};
    const auto next_epoch=s.metrics.stamp.epoch+1;
    if (next_epoch%s.audit_stride==0 || next_epoch==s.metrics.required_steps) {
        const auto copied=detail::ReadAuditConfiguration(prepared,s.audit_configuration);
        if (copied!=cudaSuccess) { s.usable=false; return {Code::StateFailure,cudaGetErrorString(copied)}; }
        double norm=0;
        if (ref::MeasureGuidedPlateStructuralNorm(*s.model,s.audit_configuration,norm,error)!=ref::ElasticCouponStatus::kSuccess)
            return {Code::AuditFailure,error};
        if (!std::isfinite(norm) || norm>s.modal.monitored_structural_norm_limit)
            return {Code::AdmissionFailure,ShellLimitDiagnostic("Guided structural operator envelope exceeded",norm,s.modal.monitored_structural_norm_limit)};
        next.last_operator_norm=norm; next.last_operator_epoch=next_epoch; ++next.full_state_audit_reads;
    }
    if (!detail::AccumulateInterval(candidate_shell,base_contact,candidate_contact,next))
        return {Code::AdmissionFailure,"Guided cumulative interval arithmetic is nonfinite"};
    GuidedPlateReport success{Code::Ok,"Accepted guided plate step"};
    state=fea::CompleteNodalValidation(s.state,token,{prepared.owner_id,prepared.kinematics.base_epoch,prepared.attempt,kGuidedPlateQualification,true});
    if (state.status!=fea::NodalStatus::Ok) return s.State(state);
    state=s.state.Commit(token); if (state.status!=fea::NodalStatus::Ok) return s.State(state);
    next.stamp=s.state.accepted(); s.metrics=next;
    s.element_association=candidate_shell; s.contact_association=candidate_contact;
    return success;
}

GuidedPlateReport GuidedPlateCase::Capture(GuidedPlateFrame& output) {
    if (!impl_) return {Code::NotInitialized,"Guided plate is not initialized"};
    auto& s=*impl_; if (!s.usable) return {Code::StateFailure,"Guided plate stopped after a CUDA failure"};
    GuidedPlateFrame candidate;
    const auto results=s.CopyResults(candidate); if (results.status!=Code::Ok) return results;
    const auto copied=s.state.CopyAccepted({candidate.position.data(),candidate.velocity.data(),ref::kCouponNodes,
        candidate.rotation.data(),candidate.omega.data(),candidate.reaction_force.data(),candidate.reaction_couple.data()},&candidate.stamp);
    if (copied.status!=fea::NodalStatus::Ok) return s.State(copied);
    if (!detail::SameStamp(candidate.stamp,s.metrics.stamp))
        return {Code::OutputFailure,"Guided accepted frame provenance changed"};
    GuidedPlateReport success{Code::Ok,"Accepted guided plate frame captured"};
    const auto shown=s.output.Publish(s.state);
    if (shown.status!=visual::Status::Ok && shown.status!=visual::Status::StaleFrame)
        return {Code::OutputFailure,shown.message};
    candidate.metrics=s.metrics; output=candidate; return success;
}
const GuidedPlateMetrics* GuidedPlateCase::metrics() const noexcept { return impl_?&impl_->metrics:nullptr; }
const ref::GuidedPlateModalReport* GuidedPlateCase::modal() const noexcept { return impl_?&impl_->modal:nullptr; }
const ref::ElasticCouponData* GuidedPlateCase::model_data() const noexcept { return impl_?&impl_->model->shell().data():nullptr; }
const ref::GuidedPlateData* GuidedPlateCase::guided_data() const noexcept { return impl_?&impl_->model->data():nullptr; }
contact::Q4PlanarReferenceView GuidedPlateCase::contact_reference() const noexcept {
    return impl_?impl_->model->contact_geometry().view():contact::Q4PlanarReferenceView{};
}
std::uint64_t GuidedPlateCase::diagnostic_stride() const noexcept { return impl_?impl_->audit_stride:0; }
const visual::NodalMeshOutput* GuidedPlateCase::output() const noexcept { return impl_?&impl_->output:nullptr; }
contact::PlanarWallView GuidedPlateCase::wall_mesh() const noexcept { return impl_?impl_->wall.view():contact::PlanarWallView{}; }
const WallProvenance* GuidedPlateCase::wall_provenance() const noexcept { return impl_?&impl_->wall.provenance:nullptr; }
fea::NodalAllocationInfo GuidedPlateCase::state_allocations() const noexcept { return impl_?impl_->state.allocations():fea::NodalAllocationInfo{}; }
fea::NodalAllocationInfo GuidedPlateCase::element_allocations() const noexcept { return impl_?impl_->elements.allocations():fea::NodalAllocationInfo{}; }
fea::NodalAllocationInfo GuidedPlateCase::contact_allocations() const noexcept { return impl_?impl_->contact.allocations():fea::NodalAllocationInfo{}; }
} // namespace crash::case_data
