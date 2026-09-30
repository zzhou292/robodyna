#include "SourceAssemblyFlightFixture.h"
#include <algorithm>
#include <limits>

namespace crash::qualification::source_assembly {
namespace {
using cases::source_assembly::test::SameBits;
void VectorBits(tl::math::Vec3 a,tl::math::Vec3 b) {SameBits(a.x,b.x);SameBits(a.y,b.y);SameBits(a.z,b.z);}
}
void CheckSource(const Rig& r) {
    const auto& b=r.bindings.shells();const auto& c=r.bindings.materials();const auto& s=r.bindings.source().data();
    EXPECT_EQ(s.identity.sha256,src::PinnedYarisSixPartInventory().sha256);
    EXPECT_EQ(s.identity.bytes,src::PinnedYarisSixPartInventory().bytes);
    ASSERT_EQ(b.node_count(),1030u);ASSERT_EQ(b.qeph_count(),804u);ASSERT_EQ(b.t3_count(),111u);
    ASSERT_EQ(c.material_count(),6u);ASSERT_EQ(c.section_count(),6u);ASSERT_EQ(c.curve_count(),2u);ASSERT_EQ(c.curve_point_count(),63u);
    ASSERT_EQ(s.parents.size(),915u);ASSERT_EQ(c.parent_count(),915u);
    ASSERT_NE(r.bindings.rigid_groups(),nullptr);EXPECT_EQ(r.bindings.rigid_groups()->group_count(),6u);
    EXPECT_EQ(r.bindings.rigid_groups()->member_count(),76u);
    const auto groups=r.owner.rigid_groups();
    EXPECT_EQ(groups.group_count,r.groups_attached?6u:0u);
    EXPECT_EQ(groups.member_count,r.groups_attached?76u:0u);
    EXPECT_EQ(groups.source_instance_id,r.groups_attached?r.bindings.source_instance_id():0u);
    EXPECT_EQ(s.boundary.policy,"released_external_connections");
    EXPECT_EQ(r.contact_geometry.binding()->inventory(),b.inventory());
    ASSERT_EQ(r.contact_geometry.weights()->node_count(),1030u);ASSERT_EQ(r.contact_geometry.weights()->parent_count(),915u);
    for(std::size_t n=0;n<r.nodes();++n) {
        EXPECT_EQ(b.nodes()[n].source_id,s.nodes[n].source_id);
        VectorBits(b.nodes()[n].position,s.nodes[n].position_m);
        const auto contact=r.contact_geometry.positions().at(n);
        SameBits(contact.x,s.nodes[n].position_m.x);SameBits(contact.y,s.nodes[n].position_m.y);SameBits(contact.z,s.nodes[n].position_m.z);
        SameBits(r.inverse_mass[n],1/b.nodes()[n].native.mass);SameBits(r.inverse_inertia[n],1/b.nodes()[n].native.isotropic_inertia);
    }
    std::size_t nq=0,nt=0;
    for(std::size_t i=0;i<s.parents.size();++i) {
        const auto& parent=s.parents[i];SCOPED_TRACE(parent.source_id);
        const auto* declared=c.parent(i);ASSERT_NE(declared,nullptr);
        const bool quad=parent.family==src::ShellFamily::Qeph;
        const auto family=quad?fe::ShellBindingFamily::Qeph:fe::ShellBindingFamily::T3;
        EXPECT_EQ(declared->family,family);EXPECT_EQ(declared->family_index,parent.family_index);
        EXPECT_EQ(declared->source_parent_id,parent.source_id);EXPECT_EQ(declared->source_part_id,parent.part_id);
        EXPECT_EQ(declared->material_id,parent.material_id);EXPECT_EQ(declared->section_id,parent.section_id);
        EXPECT_EQ(parent.source_elform,s.sections[parent.section_index].source_elform);
        EXPECT_EQ(parent.source_id,quad?b.qeph_source_id(nq++):b.t3_source_id(nt++));
        for(unsigned l=0;l<parent.arity;++l)EXPECT_EQ(parent.nodes[l],quad?b.qeph_nodes(parent.family_index)[l]:b.t3_nodes(parent.family_index)[l]);
        fe::sections::PointParameters params;ASSERT_TRUE(c.Parameters(family,parent.family_index,&params));
        const auto& material=s.materials[parent.material_index];const auto& curve=s.curves[parent.curve_index];
        SameBits(params.young_pa,material.young_pa);SameBits(params.density_kg_m3,material.density_kg_m3);
        SameBits(params.poisson_ratio,material.poisson_ratio);EXPECT_EQ(params.curve.count,curve.plastic_strain.size());
        EXPECT_EQ(curve.id,parent.curve_id);EXPECT_TRUE(params.rate.enabled);
        SameBits(params.rate.cowper_symonds_c_per_s,material.rate_c_per_s);SameBits(params.rate.cowper_symonds_p,material.rate_p);
        EXPECT_EQ(params.rate.cutoff_hz,10000); // Explicit named OpenRadiossDirectImportDefault policy.
        for(std::size_t k=0;k<curve.plastic_strain.size();++k) {
            SameBits(params.curve.plastic_strain[k],curve.plastic_strain[k]);SameBits(params.curve.yield_stress_pa[k],curve.stress_pa[k]);
        }
    }
    EXPECT_EQ(nq,r.quads());EXPECT_EQ(nt,r.triangles());
}
void CheckFlightMotion(const Rig& r,const Fields& a,const fe::ShellBatchDiagnostics& diagnostics) {
    const auto& d=r.bindings.source().data();const auto& b=r.bindings.shells();
    EXPECT_TRUE(fe::trial_identity::SameStamp(a.stamp,r.owner.accepted()));
    EXPECT_EQ(a.stamp.node_count,d.nodes.size());EXPECT_EQ(a.stamp.fixed_dt,TimeStep);
    EXPECT_EQ(a.stamp.temporal_scheme,fe::NodalTemporalScheme::StaggeredHalfKickStart);
    EXPECT_EQ(a.stamp.time,a.stamp.epoch*TimeStep);
    EXPECT_EQ(a.stamp.velocity_phase,a.stamp.epoch?fe::NodalVelocityPhase::PreviousMidpoint:fe::NodalVelocityPhase::Collocated);
    EXPECT_EQ(a.stamp.velocity_time,a.stamp.epoch?(a.stamp.epoch-.5)*TimeStep:0);
    long double total_mass=0,total_j=0,min_edge=std::numeric_limits<long double>::infinity(),coordinate=0;
    for(std::size_t n=0;n<r.nodes();++n) {
        total_mass+=b.nodes()[n].native.mass;total_j+=b.nodes()[n].native.isotropic_inertia;
        for(unsigned axis=0;axis<3;++axis)coordinate=std::max(coordinate,std::abs(static_cast<long double>(r.initial.x[3*n+axis])));
    }
    for(const auto& parent:d.parents)for(unsigned l=0;l<parent.arity;++l) {
        const auto x=d.nodes[parent.nodes[l]].position_m,y=d.nodes[parent.nodes[(l+1)%parent.arity]].position_m;
        const long double dx=static_cast<long double>(x.x)-y.x,dy=static_cast<long double>(x.y)-y.y,dz=static_cast<long double>(x.z)-y.z;
        min_edge=std::min(min_edge,std::sqrt(dx*dx+dy*dy+dz*dz));
    }
    ASSERT_GT(min_edge,0);
    // Same dimensional coefficient as the existing original-part free-flight
    // qualification, fixed before running this source assembly gate.
    const long double roundoff=2e-13L*(a.stamp.epoch+1),time=a.stamp.time;
    const long double position_bound=roundoff*(1+coordinate+Speed*time),velocity_bound=roundoff*(1+Speed);
    const long double spin_bound=velocity_bound/min_edge,orientation_bound=roundoff*(1+Speed*time/min_edge);
    for(std::size_t n=0;n<r.nodes();++n)for(unsigned axis=0;axis<3;++axis) {
        const auto j=3*n+axis;const long double v=axis==0?Speed:0;
        EXPECT_LE(std::abs(a.x[j]-(static_cast<long double>(r.initial.x[j])+time*v)),position_bound);
        EXPECT_LE(std::abs(a.v[j]-v),velocity_bound);EXPECT_LE(std::abs(a.w[j]),spin_bound);
    }
    for(std::size_t n=0;n<r.nodes();++n)for(unsigned axis=0;axis<4;++axis)
        EXPECT_LE(std::abs(a.orientation[4*n+axis]-(axis==0?1.L:0.L)),orientation_bound);
    const long double k0=.5L*total_mass*Speed*Speed,dv=std::sqrt(3.L)*velocity_bound;
    const long double allowance=total_mass*(Speed*dv+.5L*dv*dv)+1.5L*total_j*spin_bound*spin_bound+2e-12L*k0;
    EXPECT_LE(std::abs(diagnostics.kinetic.translation+diagnostics.kinetic.rotation-k0),allowance);
    const auto identity=[&](const auto& diag) {
        EXPECT_EQ(diag.owner_id,a.stamp.owner_id);EXPECT_EQ(diag.epoch,a.stamp.epoch);EXPECT_EQ(diag.time,a.stamp.time);
        EXPECT_EQ(diag.configuration_id,Configuration);EXPECT_EQ(diag.qualification_id,Qualification);EXPECT_FALSE(diag.kinetic_available);
    };
    identity(diagnostics.qeph);identity(diagnostics.t3);
}
void CheckFlight(const Rig& r,const Fields& a,const ShellFields& shells) {
    CheckFlightMotion(r,a,shells.diagnostics);
    for(const auto& h:shells.quad) {EXPECT_EQ(h.proposed_history.stamp().sample_index,a.stamp.epoch);EXPECT_EQ(h.proposed_history.stamp().time,a.stamp.time);}
    for(const auto& h:shells.triangle) {EXPECT_EQ(h.proposed_history.stamp().sample_index,a.stamp.epoch);EXPECT_EQ(h.proposed_history.stamp().time,a.stamp.time);}
    for(const auto* history:{&shells.qsection,&shells.tsection})for(const auto& h:*history) {
        EXPECT_EQ(h.cumulative_plastic_work_J,0);EXPECT_EQ(h.diagnostics.maximum_plastic_strain,0);
        for(const auto& point:h.history.point)EXPECT_EQ(point.plastic_strain,0);
    }
}
}
