#pragma once
#include "output/source_assembly/SourceAssemblyWallFields.h"
#include "output/source_assembly/WallArtifactFileIO.h"
#include "output/source_assembly/tests/SourceAssemblyOutputTestSupport.h"
#include "case/source_assembly/tests/SourceAssemblyWallSetupTestSupport.h"

namespace crash::output::assembly::test {
namespace prepared=cases::source_assembly::test;
namespace fe=tl::fea;
inline const cases::source_assembly::SourceAssemblyWallSetup& PreparedWall() {
    static const auto setup=[] {prepared::WallInput wall;cases::source_assembly::SourceAssemblyWallSetup value;
        const auto r=value.Initialize(prepared::WallAssembly(),wall.canonical,wall.bytes,prepared::WallSettings());Require(bool(r),r.message);return value;}();
    return setup;
}
inline WallArchiveRequest Request() {return {256,23,31,5,4,{}};}
inline dynamics::Config Configuration() {
    dynamics::Config c;c.fixed_dt=1./67108864;
    c.deformation={.02,1,1,.2,.2,.5,1.5,.5,1.5,.5};return c;
}
inline fe::NodalStamp Next(const fe::NodalStamp& old) {
    auto s=old;++s.epoch;s.time=old.time+old.fixed_dt;s.velocity_time=old.time+.5*old.fixed_dt;
    s.velocity_phase=fe::NodalVelocityPhase::PreviousMidpoint;s.reactions_valid=true;s.reaction_base_epoch=old.epoch;
    s.reaction_time=old.time;s.reaction_kick_dt=old.epoch?old.fixed_dt:.5*old.fixed_dt;return s;
}
// Synthetic accepted-shaped values only: they test formatting/source association,
// not a solver result. The production writer has no API to receive this fixture.
struct WallFields {
    WallFields():bindings(prepared::WallAssembly()),setup(PreparedWall()),stamp(prepared::DeclaredStamp(bindings)),
        surface(SourceAssemblySurface::Prepare(bindings.source(),{stamp.owner_id,23,31},5,bindings.source_instance_id())),
        sections(surface),x(Positions(surface)),v(3*stamp.node_count),w(v.size()),orientation(4*stamp.node_count),
        contact_nodes(stamp.node_count),contact_parents(bindings.source().data().parents.size()),faces(stamp.node_count) {
        for(std::size_t n=0;n<stamp.node_count;++n) {v[3*n]=8;orientation[4*n]=1;}
        diagnostics.stamp=stamp;auto family=[&](auto& d) {
            d.owner_id=stamp.owner_id;d.configuration_id=setup.settings()->configuration_id;d.qualification_id=setup.settings()->qualification_id;
            d.valid=true;d.phase=decltype(d.phase)::Accepted;d.usage=decltype(d.usage)::CoupledForces;d.kinetic_available=false;
        };
        family(diagnostics.shells.qeph);family(diagnostics.shells.t3);diagnostics.shells.valid=true;
        diagnostics.motion.after.phase={fe::rigid::ObservationPhaseKind::PhysicalInitialization,0,0,0};captured=diagnostics.shells;
    }
    void Interval() {
        const auto old=stamp;stamp=Next(old);diagnostics.stamp=stamp;diagnostics.has_interval=true;
        auto family=[&](auto& d) {d.epoch=stamp.epoch;d.base_epoch=old.epoch;d.attempt=7;d.time=stamp.time;d.base_time=old.time;
            d.velocity_time=stamp.velocity_time;d.base_velocity_time=old.velocity_time;d.kick_dt=stamp.reaction_kick_dt;
            d.has_completed_interval=true;d.accepted_force_assembled=true;};
        family(diagnostics.shells.qeph);family(diagnostics.shells.t3);captured=diagnostics.shells;
        diagnostics.motion.before=diagnostics.motion.after;
        diagnostics.motion.after.phase={fe::rigid::ObservationPhaseKind::StoredMidpointWithLaggedFrame,stamp.time,stamp.velocity_time,old.time};
        c.valid=true;c.phase=tlfea::contact::NodalWallDevicePhase::PreparedCandidate;c.owner_id=stamp.owner_id;
        c.configuration_id=setup.settings()->configuration_id;c.qualification_id=setup.settings()->qualification_id;c.wall_binding_id=setup.settings()->wall_binding_id;
        c.base_epoch=old.epoch;c.attempt=7;c.time=stamp.time;c.velocity_time=stamp.velocity_time;c.base_time=old.time;
        c.base_velocity_time=old.velocity_time;c.kick_dt=stamp.reaction_kick_dt;c.velocity_phase=stamp.velocity_phase;
        c.node_count=stamp.node_count;c.parent_count=contact_parents.size();
        const auto& weights=*setup.source_geometry()->weights();
        for(std::size_t i=0;i<contact_nodes.size();++i) {auto& p=contact_nodes[i];p.valid=true;p.node=i;p.base_epoch=c.base_epoch;p.attempt=c.attempt;
            p.wall_point.x=setup.placed_wall()->geometry()->wall_x();faces[i]=setup.placed_wall()->view().triangles[0].triangle_id;}
        for(std::size_t i=0;i<contact_parents.size();++i) {auto& p=contact_parents[i];const auto& expected=weights.parent(i);
            p.valid=true;p.parent_element_id=expected.parent_element_id;p.feature_id=expected.feature_id;p.parent_face_id=expected.parent_face_id;
            p.arity=expected.arity;p.family=expected.family;}
    }
    dynamics::ContactView Contact() const {return diagnostics.has_interval?dynamics::ContactView{&c,contact_parents.data(),contact_nodes.data(),faces.data(),contact_parents.size(),contact_nodes.size()}:dynamics::ContactView{};}
    wall_fields::FrameView View() const {return {&surface,&bindings,&setup,&stamp,{x.data(),v.data(),w.data(),stamp.node_count,orientation.data()},
        sections.Q(),sections.T(),&captured,&diagnostics,Contact()};}
    const cases::source_assembly::SourceAssemblyBindings& bindings;
    const cases::source_assembly::SourceAssemblyWallSetup& setup;
    fe::NodalStamp stamp;SourceAssemblySurface surface;Fields sections;std::vector<double> x,v,w,orientation;
    dynamics::Diagnostics diagnostics;fe::ShellBatchDiagnostics captured;
    tlfea::contact::NodalWallDiagnostics c;std::vector<tlfea::contact::NodalWallPointResult> contact_nodes;
    std::vector<tlfea::contact::NodalWallParentResult> contact_parents;std::vector<std::uint64_t> faces;
};
} // namespace crash::output::assembly::test
