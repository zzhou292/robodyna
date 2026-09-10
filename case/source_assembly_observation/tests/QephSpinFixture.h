#pragma once
#include "ObservationFixture.h"
#include "case/source_assembly_observation/SourceAssemblyQephSpin.h"
#include "case/source_assembly_observation/SourceAssemblyObservationInternal.h"
#include "lib_src/elements/qeph/QephHistory.h"

namespace crash::cases::source_assembly_observation::test {
// Source-bound supplied values for pure mapping tests; not an accepted CUDA trajectory.
struct SpinFixture:Fixture {
    std::vector<double> x=std::vector<double>(3*stamp.node_count),q=std::vector<double>(4*stamp.node_count);
    std::vector<fe::qeph::ForceTrial> parents=std::vector<fe::qeph::ForceTrial>(bindings.shells().qeph_count());
    std::vector<fe::ShellBatchSectionState> sections=std::vector<fe::ShellBatchSectionState>(parents.size());
    fe::qeph::BatchDiagnostics diagnostics,candidate_diagnostics;
    std::vector<fe::qeph::ForceTrial> candidate_parents;
    std::vector<fe::ShellBatchSectionState> candidate_sections;
    SpinFixture() {
        for(std::size_t n=0;n<stamp.node_count;++n) {
            const auto p=bindings.shells().nodes()[n].position;x[3*n]=p.x;x[3*n+1]=p.y;x[3*n+2]=p.z;q[4*n]=1;
        }
        for(std::size_t e=0;e<parents.size();++e)
            EXPECT_EQ(fe::qeph::InitializeHistory(bindings.shells().qeph_reference(e),{0,0},parents[e].proposed_history),fe::qeph::Status::kSuccess);
        UpdateDiagnostics();
    }
    void UpdateDiagnostics() {
        diagnostics.valid=true;diagnostics.phase=fe::qeph::BatchPhase::Accepted;diagnostics.usage=fe::qeph::BatchUsage::CoupledForces;
        diagnostics.owner_id=stamp.owner_id;diagnostics.epoch=stamp.epoch;diagnostics.time=stamp.time;diagnostics.velocity_time=stamp.velocity_time;
        diagnostics.configuration_id=71;diagnostics.qualification_id=72;
        const auto view=input().prepared;candidate_diagnostics=diagnostics;auto& d=candidate_diagnostics;
        d.phase=fe::qeph::BatchPhase::Prepared;d.base_epoch=stamp.epoch;d.epoch=stamp.epoch+1;d.attempt=view.attempt;
        d.time=view.proposed_time;d.velocity_time=view.velocity_time;d.base_time=view.base_time;
        d.base_velocity_time=view.base_velocity_time;d.kick_dt=view.kick_dt;
        candidate_parents=parents;candidate_sections=sections;
        for(std::size_t e:{392u,393u}) {
            auto& p=candidate_parents[e];const auto& ref=bindings.shells().qeph_reference(e);
            EXPECT_EQ(fe::qeph::PreparePrescribedHistory(ref,p.proposed_history.data(),{view.proposed_time,stamp.epoch+1},p.proposed_history),fe::qeph::Status::kSuccess);
            fe::qeph::PrescribedInterval interval;interval.base_time=stamp.time;interval.dt=H;interval.sample_index=stamp.epoch+1;
            for(unsigned l=0;l<4;++l) {interval.position_endpoint[l]=ref.input.position[l];interval.velocity_midpoint[l]={8,0,0};
                interval.omega_midpoint[l]=detail::Vector(old.w.data(),bindings.shells().qeph_nodes(e)[l]);}
            EXPECT_EQ(fe::qeph::EvaluatePrescribed(ref,interval,p.kinematics),fe::qeph::Status::kSuccess);
            p.internal_couple[0].x+=.123;candidate_sections[e].history.point[2].plastic_strain+=.01;
        }
    }
    void Later() {
        stamp.epoch=2;stamp.time=2*H;stamp.velocity_time=1.5*H;stamp.velocity_phase=fe::NodalVelocityPhase::PreviousMidpoint;
        stamp.reactions_valid=true;stamp.reaction_base_epoch=1;stamp.reaction_time=H;stamp.reaction_kick_dt=H;
        old.w[3*459]=3;old.w[3*459+1]=-4;old.w[3*459+2]=5;
        for(std::size_t e:{392u,393u}) {
            auto& p=parents[e];const auto& ref=bindings.shells().qeph_reference(e);auto h=p.proposed_history.data();
            h.stabilization[11]=12+e;h.material_stress[0]=101+e;
            EXPECT_EQ(fe::qeph::PreparePrescribedHistory(ref,h,{stamp.time,stamp.epoch},p.proposed_history),fe::qeph::Status::kSuccess);
            fe::qeph::PrescribedInterval interval;interval.base_time=H;interval.dt=H;interval.sample_index=2;
            for(unsigned l=0;l<4;++l) {
                interval.position_endpoint[l]=ref.input.position[l];interval.velocity_midpoint[l]={8,0,0};
                const auto n=bindings.shells().qeph_nodes(e)[l];interval.omega_midpoint[l]=detail::Vector(old.w.data(),n);
                p.internal_force[l]={double(e+1),2,3};p.internal_couple[l]={.1*e,-.2*e,.3*e};
            }
            EXPECT_EQ(fe::qeph::EvaluatePrescribed(ref,interval,p.kinematics),fe::qeph::Status::kSuccess);
            const auto local=e==392?1:0;const auto c=p.internal_couple[local];couple[3*459]-=c.x;couple[3*459+1]-=c.y;couple[3*459+2]-=c.z;
            sections[e].history.point[2].plastic_strain=.001*e;sections[e].history.point[2].stress[4]=9+e;
        }
        UpdateDiagnostics();
    }
    QephSpinInput spin_input() const {
        const auto existing=input();QephSpinInput in;in.bindings=&bindings;in.source_node=2181592;in.base=stamp;in.prepared=existing.prepared;
        in.configuration_id=71;in.qualification_id=72;
        in.before={x.data(),old.v.data(),old.w.data(),stamp.node_count,q.data()};in.parent_diagnostics=&diagnostics;in.after=in.before;in.candidate_diagnostics=&candidate_diagnostics;
        in.parents=parents.data();in.sections=sections.data();in.parent_count=parents.size();
        in.candidate_parents=candidate_parents.data();in.candidate_sections=candidate_sections.data();
        in.applied_force_xyz=force.data();in.applied_couple_xyz=couple.data();return in;
    }
};
}
