#include "AcceptedReplaySourceAssembly.h"
#include <gtest/gtest.h>
#include <cstdlib>

namespace crash::output {
namespace rd=replay_detail;
namespace {
rd::Bundle Fixture() {
    const auto* path=std::getenv("ROBO_DYNA_SOURCE_ASSEMBLY_REPLAY_FIXTURE");
    Require(path&&*path,"Explicit completed assembly fixture is required");
    rd::Bundle b;
    b.assembly=std::make_shared<rd::AssemblyReplayData>(modelio::assembly::SourceAssembly::Read(
        std::filesystem::path(path)/"source-assembly-inventory.json",modelio::assembly::PinnedYarisSixPartInventory()));
    b.info.node_count=b.assembly->source.data().nodes.size();
    b.assembly->maximum_rotation=1.;
    return b;
}
Document Config() {
    return rd::Json(R"({"rotation_domain":{"policy":"native_shell_geometry_v1",
        "native_qualification_commit":"70e7f7bf738eb087053816cc908da35a6f81399a",
        "maximum_supported_rotation_rad":1.5}})");
}
Document Diagnostic() {
    return rd::Json(R"({"maximum_rotation_rad":1.2,"native_rotation_domain":{
        "maximum_frame_rotation_rad":0.2,"maximum_nodal_normal_rotation_rad":0.3,
        "maximum_rigid_member_rotation_rad":0}})");
}
TEST(AcceptedReplaySourceAssemblyRotation,ExplicitQualificationSeparatesOrdinaryQuaternionFromNativeGeometry) {
    auto b=Fixture();auto config=Config();auto d=Diagnostic();rd::Entry e;e.epoch=1;
    EXPECT_THROW(rd::CheckAssemblyRotation(b,e,d,nullptr),std::runtime_error);
    rd::ReadAssemblyRotationDomain(*b.assembly,config);
    EXPECT_NO_THROW(rd::CheckAssemblyRotation(b,e,d,nullptr));
    for(const char* key:{"maximum_frame_rotation_rad","maximum_nodal_normal_rotation_rad","maximum_rigid_member_rotation_rad"}) {
        auto bad=Diagnostic();bad["native_rotation_domain"][key].SetDouble(1.01);
        EXPECT_THROW(rd::CheckAssemblyRotation(b,e,bad,nullptr),std::runtime_error);
    }
    d["maximum_rotation_rad"].SetDouble(std::acos(-1.)+1e-8);
    EXPECT_THROW(rd::CheckAssemblyRotation(b,e,d,nullptr),std::runtime_error);
}
TEST(AcceptedReplaySourceAssemblyRotation,UnknownMissingAndOverwideDomainDeclarationsReject) {
    auto b=Fixture();
    for(unsigned fault=0;fault<4;++fault) {
        auto c=Config();
        if(fault==0)c["rotation_domain"]["policy"].SetString("unknown",c.GetAllocator());
        if(fault==1)c["rotation_domain"]["native_qualification_commit"].SetString("unknown",c.GetAllocator());
        if(fault==2)c["rotation_domain"].RemoveMember("maximum_supported_rotation_rad");
        if(fault==3)c["rotation_domain"]["maximum_supported_rotation_rad"].SetDouble(2.);
        EXPECT_THROW(rd::ReadAssemblyRotationDomain(*b.assembly,c),std::runtime_error);
    }
    auto c=Config();b.assembly->maximum_rotation=1.5001;
    EXPECT_THROW(rd::ReadAssemblyRotationDomain(*b.assembly,c),std::runtime_error);
    b.assembly->maximum_rotation=1.;rd::ReadAssemblyRotationDomain(*b.assembly,c);
    auto d=Diagnostic();rd::Entry e;e.epoch=1;d.RemoveMember("native_rotation_domain");
    EXPECT_THROW(rd::CheckAssemblyRotation(b,e,d,nullptr),std::runtime_error);
    d=Diagnostic();e.epoch=0;
    EXPECT_THROW(rd::CheckAssemblyRotation(b,e,d,nullptr),std::runtime_error);
}
TEST(AcceptedReplaySourceAssemblyRotation,ActualRigidMembershipCannotBeBypassedByRelabelingDiagnostics) {
    auto b=Fixture();auto config=Config();rd::ReadAssemblyRotationDomain(*b.assembly,config);
    b.assembly->grouped_node.assign(b.info.node_count,false);b.assembly->grouped_node[1]=true;
    Document nodal;nodal.SetObject();Value q(rapidjson::kArrayType);
    for(std::size_t n=0;n<b.info.node_count;++n)
        for(unsigned j=0;j<4;++j)q.PushBack(j==0?1.:0.,nodal.GetAllocator());
    nodal.AddMember("orientation_wxyz",q,nodal.GetAllocator());
    const double w=std::cos(.6),x=std::sin(.6);
    nodal["orientation_wxyz"][0].SetDouble(w);nodal["orientation_wxyz"][1].SetDouble(x);
    auto d=Diagnostic();d["maximum_rotation_rad"].SetDouble(2*std::atan2(std::hypot(std::hypot(x,0.),0.),std::abs(w)));
    rd::Entry e;e.epoch=1;
    EXPECT_NO_THROW(rd::CheckAssemblyRotation(b,e,d,&nodal));
    nodal["orientation_wxyz"][4].SetDouble(w);nodal["orientation_wxyz"][5].SetDouble(x);
    EXPECT_THROW(rd::CheckAssemblyRotation(b,e,d,&nodal),std::runtime_error);
}
} // namespace
} // namespace crash::output
