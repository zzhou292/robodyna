#include "ElasticCouponCase.h"
#include "lib_src/solvers/ExplicitNodalStep.h"
#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>

namespace crash::case_data {
namespace fea=tl::fea;
namespace shell=tl::fea::reissner;
namespace ref=crash::reference;
namespace {
constexpr std::uint64_t kQualification=0x4232454c41535449ULL; // B2 elastic experiment revision.
std::string BatchDiagnostic(const shell::ShellBatchReport& r) {
    return std::string(r.message)+"; batch_status="+std::to_string(static_cast<int>(r.status))+
        ", element="+std::to_string(r.element)+", node="+std::to_string(r.node)+
        ", element_status="+std::to_string(static_cast<int>(r.element_status));
}
bool Matches(const shell::ShellBatchDiagnostics& d,const fea::NodalPreparedView& p) {
    return d.valid && d.phase==shell::ShellBatchPhase::kPreparedCandidate &&
           d.owner_id==p.owner_id && d.base_epoch==p.kinematics.base_epoch &&
           d.attempt==p.attempt && d.configuration_id==kQualification;
}
visual::Binding Binding(const fea::FENodalState& owner,const ref::ElasticCouponData& data) {
    visual::Binding b; b.identity={owner.accepted().owner_id,1,kQualification}; b.tl_node_count=ref::kCouponNodes;
    for(unsigned n=0;n<ref::kCouponNodes;++n) b.vertices.push_back({n,{1,1,n+1}});
    for(unsigned e=0;e<ref::kCouponElements;++e) {
        std::array<std::uint32_t,4> n{};
        for(unsigned i=0;i<4;++i) n[i]=static_cast<std::uint32_t>(data.connectivity[e][i]);
        b.triangles.push_back({{n[0],n[1],n[2]},1,1,e+1,1,0,0});
        b.triangles.push_back({{n[0],n[2],n[3]},1,1,e+1,1,0,1});
    }
    return b;
}
}  // namespace

struct ElasticCouponCase::Impl {
    ref::ElasticCouponModel model;
    ref::ElasticCouponModalReport modal;
    fea::FENodalState state;
    shell::ReissnerShellBatch batch;
    visual::NodalMeshOutput output;
    ElasticCouponMetrics metrics;
    shell::ShellBatchDiagnostics element_association;
    ref::ElasticCouponConfiguration audit_configuration;
    std::uint64_t audit_stride=1;
    bool usable=true;
};

ElasticCouponCase::ElasticCouponCase()=default;
ElasticCouponCase::~ElasticCouponCase()=default;
CouponReport ElasticCouponCase::Initialize(const ElasticCouponConfig& config) {
    if(impl_) return {CouponStatus::AlreadyInitialized,"Coupon is already initialized"};
    if((config.refinement!=1 && config.refinement!=2 && config.refinement!=4) ||
       !config.diagnostic_intervals || config.diagnostic_intervals>64)
        return {CouponStatus::InvalidInput,"Invalid fixed-step refinement or diagnostic cadence"};
    try {
        auto s=std::make_unique<Impl>(); std::string error;
        if(ref::AuditElasticCoupon(s->model,s->modal,error)!=ref::ElasticCouponStatus::kSuccess)
            return {CouponStatus::AuditFailure,error};
        if(!s->modal.step_count || s->modal.step_count>100000)
            return {CouponStatus::AdmissionFailure,"Coupon half-period exceeds the admitted initial step-count budget"};
        const auto& data=s->model.data();
        std::array<double,3*ref::kCouponNodes> x{},v{},omega{};
        std::array<double,4*ref::kCouponNodes> q{};
        std::array<std::uint8_t,ref::kCouponNodes> translation{},rotation{};
        for(unsigned i=0;i<ref::kCouponNodes;++i) {
            const auto p=s->modal.initial_configuration.position[i];
            const auto r=s->modal.initial_configuration.rotation[i];
            x[3*i]=p.x; x[3*i+1]=p.y; x[3*i+2]=p.z;
            q[4*i]=r.w; q[4*i+1]=r.x; q[4*i+2]=r.y; q[4*i+3]=r.z;
            translation[i]=data.fixed[i]?7:0; rotation[i]=data.fixed[i]?1:0;
        }
        fea::NodalStateConfig state_config; state_config.node_count=ref::kCouponNodes;
        state_config.fixed_dt=s->modal.time_step/config.refinement;
        auto state_report=s->state.Initialize(state_config,{x.data(),v.data(),omega.data(),ref::kCouponNodes,q.data()},
            data.inverse_mass.data(),{translation.data(),rotation.data(),data.inverse_isotropic_inertia.data()});
        if(state_report.status!=fea::NodalStatus::Ok) return {CouponStatus::StateFailure,state_report.message};
        std::array<shell::ReissnerShellBatchElement,ref::kCouponElements> elements;
        for(unsigned e=0;e<ref::kCouponElements;++e) {
            elements[e].reference=data.reference[e]; elements[e].section=data.section[e];
            for(unsigned i=0;i<4;++i) elements[e].nodes[i]=data.connectivity[e][i];
        }
        shell::ReissnerShellBatchConfig batch_config;
        batch_config.owner=s->state.accepted(); batch_config.configuration_id=kQualification;
        batch_config.element_count=ref::kCouponElements;
        batch_config.drilling_policy=shell::ShellDrillingInertiaPolicy::kEqualPhysicalTangential;
        auto batch_report=s->batch.Initialize(batch_config,elements.data());
        if(batch_report.status!=shell::ShellBatchStatus::kSuccess) return {CouponStatus::ElementFailure,BatchDiagnostic(batch_report)};
        fea::NodalTrialToken token; fea::NodalAssemblyView view;
        state_report=s->state.BeginTrial(&token,&view);
        if(state_report.status!=fea::NodalStatus::Ok) return {CouponStatus::StateFailure,state_report.message};
        batch_report=s->batch.Assemble(view,&s->metrics.diagnostics); s->state.Discard();
        if(batch_report.status!=shell::ShellBatchStatus::kSuccess) return {CouponStatus::ElementFailure,BatchDiagnostic(batch_report)};
        const auto& initial=s->metrics.diagnostics;
        s->element_association=initial;
        s->metrics.initial_energy=initial.elastic_energy+CouponKineticEnergy(initial);
        if(!CheckElasticCouponEnvelope(initial,s->modal,s->metrics.initial_energy,
                                      ElasticCouponLimits::displacement,false,error))
            return {CouponStatus::AdmissionFailure,error};
        if(std::abs(initial.elastic_energy-s->modal.initial_energy)>1e-8*s->metrics.initial_energy)
            return {CouponStatus::AuditFailure,"Initial CPU/CUDA elastic energy disagreement"};
        s->metrics.stamp=s->state.accepted(); s->metrics.required_steps=s->modal.step_count*config.refinement;
        s->metrics.last_operator_norm=s->modal.sampled_operator_norm[2];
        s->audit_stride=((s->modal.step_count+config.diagnostic_intervals-1)/config.diagnostic_intervals)*config.refinement;
        auto output=s->output.Initialize(s->state,Binding(s->state,data));
        if(output.status!=visual::Status::Ok) return {CouponStatus::OutputFailure,output.message};
        output=s->output.Publish(s->state);
        if(output.status!=visual::Status::Ok) return {CouponStatus::OutputFailure,output.message};
        impl_=std::move(s); return {CouponStatus::Ok,"Elastic coupon initialized"};
    } catch(const std::exception& e) { return {CouponStatus::InvalidInput,e.what()}; }
}

CouponReport ElasticCouponCase::Step(const ElasticCouponStepRequest& request) {
    if(!impl_) return {CouponStatus::NotInitialized,"Coupon is not initialized"};
    auto& s=*impl_;
    if(!s.usable) return {CouponStatus::StateFailure,"Coupon stopped after a CUDA readback failure"};
    if(!std::isfinite(request.maximum_displacement) || request.maximum_displacement<=0 ||
       request.maximum_displacement>ElasticCouponLimits::displacement ||
       s.metrics.stamp.epoch>=s.metrics.required_steps)
        return {CouponStatus::InvalidInput,"Invalid coupon stop envelope or completed admitted horizon"};
    auto reject=[&](CouponStatus status,const std::string& error) { s.state.Discard(); return CouponReport{status,error}; };
    fea::NodalTrialToken token; fea::NodalAssemblyView view;
    auto state_report=s.state.BeginTrial(&token,&view);
    if(state_report.status!=fea::NodalStatus::Ok) return reject(CouponStatus::StateFailure,state_report.message);
    shell::ShellBatchDiagnostics base,candidate;
    auto batch_report=s.batch.Assemble(view,&base);
    if(batch_report.status!=shell::ShellBatchStatus::kSuccess) return reject(CouponStatus::ElementFailure,BatchDiagnostic(batch_report));
    state_report=s.state.SealAssembly(token);
    if(state_report.status!=fea::NodalStatus::Ok) return reject(CouponStatus::StateFailure,state_report.message);
    fea::NodalStepAdmission admission{view.owner_id,view.accepted.base_epoch,view.attempt,s.modal.proposed_step_limit,
        ElasticCouponLimits::rotation_increment,fea::NodalStepAdmissionKind::RestrictedElasticTrajectory,
        kQualification,s.modal.monitored_norm_limit};
    state_report=fea::AdvanceNodal(s.state,token,admission);
    if(state_report.status!=fea::NodalStatus::Ok) return reject(CouponStatus::StateFailure,state_report.message);
    fea::NodalPreparedView prepared;
    state_report=s.state.BorrowPrepared(token,&prepared);
    if(state_report.status!=fea::NodalStatus::Ok) return reject(CouponStatus::StateFailure,state_report.message);
    batch_report=s.batch.EvaluateCandidate(prepared,&candidate);
    if(batch_report.status!=shell::ShellBatchStatus::kSuccess) return reject(CouponStatus::ElementFailure,BatchDiagnostic(batch_report));
    if(!Matches(candidate,prepared)) return reject(CouponStatus::AdmissionFailure,"Candidate diagnostic association mismatch");
    std::string error;
    if(!CheckElasticCouponEnvelope(candidate,s.modal,s.metrics.initial_energy,request.maximum_displacement,true,error))
        return reject(CouponStatus::AdmissionFailure,error);
    auto next=s.metrics; const auto next_epoch=s.metrics.stamp.epoch+1;
    if(next_epoch%s.audit_stride==0 || next_epoch==s.metrics.required_steps) {
        static_assert(sizeof(shell::Vec3)==3*sizeof(double) && sizeof(shell::Quaternion)==4*sizeof(double));
        auto copy=cudaMemcpyAsync(s.audit_configuration.position.data(),prepared.kinematics.position_xyz,
            sizeof(s.audit_configuration.position),cudaMemcpyDeviceToHost,prepared.stream);
        if(copy==cudaSuccess) copy=cudaMemcpyAsync(s.audit_configuration.rotation.data(),prepared.kinematics.orientation_wxyz,
            sizeof(s.audit_configuration.rotation),cudaMemcpyDeviceToHost,prepared.stream);
        if(copy==cudaSuccess) copy=cudaStreamSynchronize(prepared.stream);
        if(copy!=cudaSuccess) { s.usable=false; return reject(CouponStatus::StateFailure,cudaGetErrorString(copy)); }
        double norm=0;
        if(ref::MeasureElasticCouponOperatorNorm(s.model,s.audit_configuration,norm,error)!=ref::ElasticCouponStatus::kSuccess)
            return reject(CouponStatus::AuditFailure,error);
        if(!std::isfinite(norm) || norm>s.modal.monitored_norm_limit)
            return reject(CouponStatus::AdmissionFailure,CouponLimitDiagnostic("Coupon sampled spectral envelope exceeded",norm,s.modal.monitored_norm_limit));
        next.last_operator_norm=norm; next.last_operator_epoch=next_epoch; ++next.full_state_audit_reads;
    }
    next.diagnostics=candidate;
    const double energy=candidate.elastic_energy+CouponKineticEnergy(candidate);
    next.maximum_relative_energy_error=std::max(next.maximum_relative_energy_error,
                                               std::abs(energy-next.initial_energy)/next.initial_energy);
    state_report=fea::CompleteNodalValidation(s.state,token,{view.owner_id,view.accepted.base_epoch,view.attempt,kQualification,true});
    if(state_report.status!=fea::NodalStatus::Ok) return reject(CouponStatus::StateFailure,state_report.message);
    state_report=s.state.Commit(token);
    if(state_report.status!=fea::NodalStatus::Ok) return reject(CouponStatus::StateFailure,state_report.message);
    next.stamp=s.state.accepted(); s.metrics=next;
    s.element_association=candidate;
    return {CouponStatus::Ok,"Accepted elastic coupon step"};
}

CouponReport ElasticCouponCase::Capture(ElasticCouponFrame& output) {
    if(!impl_) return {CouponStatus::NotInitialized,"Coupon is not initialized"};
    auto& s=*impl_; ElasticCouponFrame candidate;
    auto batch=s.batch.CopyElementResults(s.element_association,candidate.element.data(),candidate.element.size());
    if(batch.status==shell::ShellBatchStatus::kStaleTrial) {
        // A failed attempt can replace result scratch, never accepted state.
        // Reevaluate that accepted geometry only at the requested output cadence.
        fea::NodalTrialToken token; fea::NodalAssemblyView view;
        const auto begin=s.state.BeginTrial(&token,&view);
        if(begin.status!=fea::NodalStatus::Ok) return {CouponStatus::StateFailure,begin.message};
        shell::ShellBatchDiagnostics refreshed;
        batch=s.batch.Assemble(view,&refreshed);
        if(batch.status==shell::ShellBatchStatus::kSuccess)
            batch=s.batch.CopyElementResults(refreshed,candidate.element.data(),candidate.element.size());
        s.state.Discard();
        if(batch.status==shell::ShellBatchStatus::kSuccess) s.element_association=refreshed;
    }
    if(batch.status!=shell::ShellBatchStatus::kSuccess) return {CouponStatus::OutputFailure,BatchDiagnostic(batch)};
    const auto state=s.state.CopyAccepted({candidate.position.data(),candidate.velocity.data(),ref::kCouponNodes,
        candidate.rotation.data(),candidate.omega.data(),candidate.reaction_force.data(),candidate.reaction_couple.data()},&candidate.stamp);
    if(state.status!=fea::NodalStatus::Ok) return {CouponStatus::StateFailure,state.message};
    if(candidate.stamp.owner_id!=s.metrics.stamp.owner_id || candidate.stamp.epoch!=s.metrics.stamp.epoch)
        return {CouponStatus::OutputFailure,"Accepted frame provenance changed"};
    const auto published=s.output.Publish(s.state);
    if(published.status!=visual::Status::Ok && published.status!=visual::Status::StaleFrame)
        return {CouponStatus::OutputFailure,published.message};
    candidate.metrics=s.metrics; candidate.element_association=s.element_association; output=candidate;
    return {CouponStatus::Ok,"Accepted elastic coupon frame captured"};
}
const ElasticCouponMetrics* ElasticCouponCase::metrics() const noexcept { return impl_?&impl_->metrics:nullptr; }
const ref::ElasticCouponModalReport* ElasticCouponCase::modal() const noexcept { return impl_?&impl_->modal:nullptr; }
const ref::ElasticCouponData* ElasticCouponCase::model_data() const noexcept { return impl_?&impl_->model.data():nullptr; }
const visual::NodalMeshOutput* ElasticCouponCase::output() const noexcept { return impl_?&impl_->output:nullptr; }
fea::NodalAllocationInfo ElasticCouponCase::state_allocations() const noexcept { return impl_?impl_->state.allocations():fea::NodalAllocationInfo{}; }
fea::NodalAllocationInfo ElasticCouponCase::element_allocations() const noexcept { return impl_?impl_->batch.allocations():fea::NodalAllocationInfo{}; }
}  // namespace crash::case_data
