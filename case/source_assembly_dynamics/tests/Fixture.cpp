#include "Fixture.h"
#include "case/CanonicalWallArtifacts.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <cstdlib>
#include <cstring>
#include <sstream>

namespace crash::cases::source_assembly_dynamics::test {
Config SmokeConfig() {
    Config c;c.fixed_dt=Dt;
    auto& d=c.deformation;d.maximum_displacement=.02;d.maximum_rotation=1;d.maximum_rotation_increment=1;
    d.maximum_strain=.2;d.maximum_thickness_curvature=.2;d.minimum_area_ratio=.5;d.maximum_area_ratio=1.5;
    d.minimum_thickness_ratio=.5;d.maximum_thickness_ratio=1.5;d.maximum_native_dt_fraction=.5;
    return c;
}
source::SourceAssemblyWallSetup PrepareWall(const source::SourceAssemblyBindings& bindings,double gap) {
    const auto* path=std::getenv("ROBO_DYNA_SOURCE_ASSEMBLY_WALL");
    output::Require(path&&*path,"Explicit original finite wall fixture is required");
    const auto bytes=case_data::ReadPinnedWallManifest(path);case_data::CanonicalWall wall;std::istringstream input(bytes);
    output::Require(wall.Load(input).status==case_data::WallStatus::Ok,"Original wall fixture failed");
    source::SourceAssemblyWallSettings settings;settings.initial_velocity={8,0,0};settings.leading_gap=gap;
    settings.configuration_id=0x534157434631ULL;settings.qualification_id=0x534157434751ULL;settings.wall_binding_id=0x53415757414cULL;
    settings.boundary=source::SourceAssemblyWallBoundary::ReleasedExternalConnections;
    source::SourceAssemblyWallSetup result;const auto r=result.Initialize(bindings,wall,bytes,settings);
    output::Require(bool(r),r.message);return result;
}
void SameFields(const Fields& a,const Fields& b) {
    EXPECT_EQ(a.x,b.x);EXPECT_EQ(a.v,b.v);EXPECT_EQ(a.w,b.w);EXPECT_EQ(a.orientation,b.orientation);
    EXPECT_EQ(a.reaction,b.reaction);EXPECT_EQ(a.couple,b.couple);
    ASSERT_EQ(a.groups.size(),b.groups.size());
    EXPECT_EQ(std::memcmp(a.groups.data(),b.groups.data(),a.groups.size()*sizeof(fe::NodalRigidGroupSnapshot)),0);
}
void SameParents(const ParentFields& a,const ParentFields& b) {
    ASSERT_EQ(a.qeph.size(),b.qeph.size());ASSERT_EQ(a.t3.size(),b.t3.size());
    EXPECT_EQ(std::memcmp(a.qeph.data(),b.qeph.data(),a.qeph.size()*sizeof(q::ForceTrial)),0);
    EXPECT_EQ(std::memcmp(a.t3.data(),b.t3.data(),a.t3.size()*sizeof(t::ForceTrial)),0);
    EXPECT_EQ(std::memcmp(a.qsection.data(),b.qsection.data(),a.qsection.size()*sizeof(fe::ShellBatchSectionState)),0);
    EXPECT_EQ(std::memcmp(a.tsection.data(),b.tsection.data(),a.tsection.size()*sizeof(fe::ShellBatchSectionState)),0);
}
void SameAccepted(const Sample& a,const Sample& b) {
    SameFields(a.fields,b.fields);SameParents(a.parents,b.parents);
    EXPECT_EQ(std::memcmp(&a.diagnostics,&b.diagnostics,sizeof(Diagnostics)),0);
    EXPECT_EQ(std::memcmp(&a.wall.diagnostics,&b.wall.diagnostics,sizeof(contact::NodalWallDiagnostics)),0);
    EXPECT_EQ(std::memcmp(a.wall.parents.data(),b.wall.parents.data(),a.wall.parents.size()*sizeof(contact::NodalWallParentResult)),0);
    EXPECT_EQ(std::memcmp(a.wall.nodes.data(),b.wall.nodes.data(),a.wall.nodes.size()*sizeof(contact::NodalWallPointResult)),0);
    EXPECT_EQ(a.wall.wall_face,b.wall.wall_face);EXPECT_EQ(a.qwork_magnitude,b.qwork_magnitude);EXPECT_EQ(a.twork_magnitude,b.twork_magnitude);
}
void SameAllocations(const SourceAssemblyWallCase& c,fe::NodalAllocationInfo old,std::size_t host) {
    EXPECT_EQ(c.allocations().device_bytes,old.device_bytes);EXPECT_EQ(c.allocations().device_allocations,old.device_allocations);
    EXPECT_EQ(c.host_payload_bytes(),host);
}
void CheckOutput(SourceAssemblyWallCase& run,SourceAssemblyAcceptedOutput& output) {
    const auto report=run.CaptureAccepted(output);ASSERT_TRUE(report)<<report.message;
    const auto& sample=SourceAssemblyDynamicsTestAccess::Accepted(run);const auto* stamp=output.nodal()->stamp();
    ASSERT_NE(stamp,nullptr);EXPECT_TRUE(fe::trial_identity::SameStamp(*stamp,run.diagnostics()->stamp));
    const auto fields=output.nodal()->fields();ASSERT_EQ(fields.node_count,1030u);
    EXPECT_EQ(std::memcmp(fields.position_xyz,sample.fields.x.data(),sample.fields.x.size()*sizeof(double)),0);
    EXPECT_EQ(std::memcmp(fields.velocity_xyz,sample.fields.v.data(),sample.fields.v.size()*sizeof(double)),0);
    EXPECT_EQ(std::memcmp(fields.angular_velocity_xyz,sample.fields.w.data(),sample.fields.w.size()*sizeof(double)),0);
    EXPECT_EQ(std::memcmp(fields.orientation_wxyz,sample.fields.orientation.data(),sample.fields.orientation.size()*sizeof(double)),0);
    ASSERT_EQ(output.qeph().count,804u);ASSERT_EQ(output.t3().count,111u);
    EXPECT_EQ(std::memcmp(output.qeph().values,sample.parents.qsection.data(),804*sizeof(fe::ShellBatchSectionState)),0);
    EXPECT_EQ(std::memcmp(output.t3().values,sample.parents.tsection.data(),111*sizeof(fe::ShellBatchSectionState)),0);
    ASSERT_EQ(output.nodal()->surface().mesh()->GetCoordsVertices().size(),1030u);
    ASSERT_EQ(output.nodal()->surface().mesh()->GetIndicesVertices().size(),1719u);
    EXPECT_EQ(output.parent_scalars()->size(),915u);
}
} // namespace crash::cases::source_assembly_dynamics::test
