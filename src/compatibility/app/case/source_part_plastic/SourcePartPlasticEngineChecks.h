#pragma once
#include "case/source_part_elastic/SourcePartWallEngineTestSupport.h"
#include <cstring>

namespace crash::cases::source_part_plastic::engine_check {
namespace part = source_part_elastic;
namespace fe = tl::fea;
using source_part_wall::check::SameValue;

template<class Range> void SameArray(const Range& a,const Range& b) {
    for(std::size_t i=0;i<std::size(a);++i) { SCOPED_TRACE(i); SameValue(a[i],b[i]); }
}
inline void SameSummary(const PlasticSummary& a,const PlasticSummary& b) {
    SameValue(a.enabled,b.enabled); SameValue(a.maximum_plastic_strain,b.maximum_plastic_strain);
    SameValue(a.mean_plastic_strain,b.mean_plastic_strain);
    SameValue(a.cumulative_plastic_work_J,b.cumulative_plastic_work_J);
    SameValue(a.yielded_points,b.yielded_points); SameValue(a.yielded_parents,b.yielded_parents);
}
inline void SameSectionDiagnostics(const fe::sections::ShellLayeredJ2Diagnostics& a,
                                   const fe::sections::ShellLayeredJ2Diagnostics& b) {
#define SAME(field) SameValue(a.field,b.field)
    SAME(plastic_work_density_increment); SAME(maximum_plastic_strain); SAME(mean_plastic_strain);
    SAME(minimum_tangent_ratio); SAME(mean_tangent_ratio); SAME(mean_yield_before_pa); SAME(last_point_yield_before_pa);
#undef SAME
}
inline void SameSections(const SourcePartPlasticState& a,const SourcePartPlasticState& b) {
    const auto compare=[](const auto& x,const auto& y) {
        for(unsigned e=0;e<x.size();++e) {
            SCOPED_TRACE(e);
            for(unsigned p=0;p<3;++p) {
                SameValue(x[e].history.point[p].plastic_strain,y[e].history.point[p].plastic_strain);
                SameValue(x[e].history.point[p].filtered_rate_per_s,y[e].history.point[p].filtered_rate_per_s);
                SameArray(x[e].history.point[p].stress,y[e].history.point[p].stress);
            }
            SameSectionDiagnostics(x[e].diagnostics,y[e].diagnostics);
            SameValue(x[e].cumulative_plastic_work_J,y[e].cumulative_plastic_work_J);
        }
    };
    compare(a.qeph,b.qeph); compare(a.t3,b.t3);
    SameArray(a.qeph_reported_thickness,b.qeph_reported_thickness);
    SameArray(a.t3_reported_thickness,b.t3_reported_thickness);
}
template<class D> void SameFamily(const D& a,const D& b,bool retry) {
#define SAME(field) SameValue(a.field,b.field)
    SAME(owner_id); SAME(configuration_id); SAME(qualification_id); SAME(epoch); SAME(base_epoch);
    SAME(time); SAME(base_time); SAME(velocity_time); SAME(base_velocity_time); SAME(kick_dt);
    if(retry) { EXPECT_GT(b.attempt,a.attempt); }
    else { SAME(attempt); SAME(phase); }
    SAME(valid); SAME(has_completed_interval); SAME(accepted_force_assembled); SAME(kinetic_available); SAME(usage);
    SAME(kinetic_translation); SAME(kinetic_rotation); SAME(kinetic_physical_isotropic); SAME(kinetic_added_isotropic);
    SameArray(a.internal_work,b.internal_work); SameArray(a.internal_work_increment,b.internal_work_increment);
    SAME(minimum_area_ratio); SAME(minimum_thickness_ratio); SAME(maximum_displacement);
    SAME(maximum_absolute_strain); SAME(maximum_thickness_curvature); SAME(minimum_native_dt);
    SAME(internal_kick_work); SAME(internal_drift_work);
#undef SAME
}
inline void SameKinetic(const fe::ShellBatchKinetic& a,const fe::ShellBatchKinetic& b) {
    SameValue(a.translation,b.translation); SameValue(a.rotation,b.rotation);
    SameValue(a.physical_isotropic,b.physical_isotropic); SameValue(a.added_isotropic,b.added_isotropic);
}
inline void SameSnapshot(const part::Snapshot& a,const part::Snapshot& b,bool retry=false) {
    SameArray(a.position,b.position); SameArray(a.velocity,b.velocity); SameArray(a.omega,b.omega);
    SameArray(a.orientation,b.orientation); SameArray(a.synchronized_velocity,b.synchronized_velocity);
    SameArray(a.synchronized_omega,b.synchronized_omega); SameSummary(a.plastic,b.plastic);
    if(!retry) {
        const auto& x=a.stamp; const auto& y=b.stamp;
#define SAME(field) SameValue(x.field,y.field)
        SAME(owner_id); SAME(epoch); SAME(node_count); SAME(time); SAME(fixed_dt); SAME(has_rotations);
        SAME(reactions_valid); SAME(reaction_base_epoch); SAME(reaction_time); SAME(temporal_scheme);
        SAME(velocity_phase); SAME(velocity_time); SAME(reaction_kick_dt);
#undef SAME
    }
    const auto& x=a.diagnostics; const auto& y=b.diagnostics;
#define SAME(field) SameValue(x.field,y.field)
    SAME(external_kick_work); SAME(external_drift_work); SAME(absolute_external_drift_work);
    SAME(kinetic_work_residual); SAME(kinetic_work_allowance); SAME(synchronized_kinetic); SAME(total_internal_work);
    SAME(energy_residual); SAME(max_relative_displacement); SAME(maximum_rotation);
    SAME(maximum_area_ratio); SAME(maximum_thickness_ratio);
    if(!retry) { SAME(maximum_chord_change); }
    // Capture computes chord change; the clean prepared snapshot has no output-cadence metric.
    SAME(shells.valid);
#undef SAME
    SameFamily(x.shells.qeph,y.shells.qeph,retry); SameFamily(x.shells.t3,y.shells.t3,retry);
    SameValue(x.shells.qeph.hourglass_viscous_work,y.shells.qeph.hourglass_viscous_work);
    SameValue(x.shells.qeph.hourglass_viscous_work_increment,y.shells.qeph.hourglass_viscous_work_increment);
    SameKinetic(x.shells.base_kinetic,y.shells.base_kinetic); SameKinetic(x.shells.kinetic,y.shells.kinetic);
    if(retry) {
        EXPECT_EQ(y.shells.qeph.phase,fe::qeph::BatchPhase::Accepted);
        EXPECT_EQ(y.shells.t3.phase,fe::t3::BatchPhase::Accepted);
    }
}
inline void ReadShells(part::SourcePartElasticCase& run,part::test::Results& out) {
    auto& p=part::SourcePartElasticTestAccess::Internal(run);
    fe::qeph::BatchDiagnostics qd; fe::t3::BatchDiagnostics td;
    ASSERT_EQ(p.qeph.CopyAcceptedResults(run.owner().accepted(),out.qeph.data(),out.qeph.size(),&qd).status,
              fe::qeph::BatchStatus::Success);
    ASSERT_EQ(p.t3.CopyAcceptedResults(run.owner().accepted(),out.t3.data(),out.t3.size(),&td).status,
              fe::t3::BatchStatus::Success);
}
inline void SameShells(const part::test::Results& a,const part::test::Results& b) {
    // QEPH's initialized device slabs already have a qualified byte-exact retry
    // contract. T3 uses typed fields to avoid depending on its structure padding.
    EXPECT_EQ(std::memcmp(a.qeph.data(),b.qeph.data(),sizeof(a.qeph)),0);
    for(unsigned e=0;e<a.t3.size();++e) {
        SCOPED_TRACE(e);
        const auto& x=a.t3[e]; const auto& y=b.t3[e];
        t3_port_test::ExactRates(x.kinematics,y.kinematics);
        const auto& h=x.proposed_history.data(); const auto& k=y.proposed_history.data();
        SameArray(h.stress,k.stress); SameArray(h.material_stress,k.material_stress);
        SameArray(h.bending_stress,k.bending_stress); SameArray(h.strain_curvature,k.strain_curvature);
        SameArray(h.internal_work,k.internal_work); SameValue(h.thickness,k.thickness);
        SameValue(h.equivalent_strain_rate,k.equivalent_strain_rate); SameValue(h.active,k.active);
        SameValue(x.proposed_history.prepared(),y.proposed_history.prepared());
        SameValue(x.proposed_history.stamp().time,y.proposed_history.stamp().time);
        SameValue(x.proposed_history.stamp().sample_index,y.proposed_history.stamp().sample_index);
        for(unsigned n=0;n<3;++n) {
            for(unsigned c=0;c<3;++c) {
                SameValue(t3_port_test::Component(x.internal_force[n],c),t3_port_test::Component(y.internal_force[n],c));
                SameValue(t3_port_test::Component(x.internal_couple[n],c),t3_port_test::Component(y.internal_couple[n],c));
            }
        }
        SameArray(t3_force_port_test::Diagnostics(x.diagnostics),t3_force_port_test::Diagnostics(y.diagnostics));
    }
}
}  // namespace crash::cases::source_part_plastic::engine_check
