#include "../Internal.h"
#include "lib_utest/qualification/radioss_type25_fixed_main_startup/NativeOracle.h"
#include "lib_utest/qualification/radioss_type25_fixed_main_startup/Assertions.h"
namespace crash::cases::vehicle_wall::native::wall_interface::test {
TEST(FiniteWallTopology, BothPhasesMatchWholeNativeOnCompleteDomainWithUnusedVehicleNodes) {
    namespace d=detail;
    d::Fields fields;d::Plan plan;plan.shape.nodes=20;plan.shape.vehicle_nodes=16;
    plan.shape.units={.001,1000,1};plan.shape.wall_nodes={16,17,18,19};
    plan.topology=s::Preflight(20,1,plan.topology_limits);
    plan.forecast.own_retained=1u<<20;plan.forecast.topology_scratch=std::max(plan.topology.scratch_bytes,plan.topology.ready_scratch_bytes);
    d::Allocate(fields,plan);
    for(unsigned i=0;i<20;++i){fields.ids[i]=100+i;fields.positions[3*i]=double(i);fields.positions[3*i+1]=0;fields.positions[3*i+2]=0;}
    const double xyz[]{10,-2,-3,10,2,-3,10,2,3,10,-2,3};
    std::copy_n(xyz,12,fields.positions.begin()+48);
    fields.primary.source_id=71;fields.primary.layout=n::ShellLayout::Quad4;
    for(unsigned k=0;k<4;++k)fields.primary.nodes[k]=16+k;
    fields.main_k={2e5,2e5};const Declaration declaration{Profile::AllRetainedVehicleNodesToFixedMeshV1,7,9};
    const auto mesh=fields.Mesh(declaration,plan.shape.units);
    ASSERT_TRUE(fields.topology_arena.Initialize(plan.topology.output_bytes));
    ASSERT_TRUE(fields.ready_arena.Initialize(plan.topology.ready_output_bytes));
    tl::util::HostArena scratch;ASSERT_TRUE(scratch.Initialize(plan.forecast.topology_scratch));
    ASSERT_EQ(s::BuildStarter(mesh,plan.topology_limits,fields.topology_arena,scratch,&fields.starter).status,s::Status::Ok);
    ASSERT_EQ(s::BuildFixedMain(mesh,fields.starter,{fields.main_k.data(),2},plan.topology_limits,fields.ready_arena,scratch,&fields.ready).status,s::Status::Ok);
    const auto native=type25_startup_test::Oracle(mesh,fields.main_k.data(),2);
    type25_startup_test::SameStarter(fields.starter,native);
    for(std::size_t i=0;i<8;++i)type25_startup_test::Same(fields.ready.normals.face_normals[i],native.ready_normals[i]);
    for(std::size_t i=0;i<fields.ready.normals.reference_count;++i) {
        EXPECT_EQ(fields.ready.normals.references[i].boundary,native.ready_references[i].boundary);
        for(unsigned k=0;k<2;++k)type25_startup_test::Same(fields.ready.normals.references[i].bisector[k],native.ready_references[i].bisector[k]);
    }
    EXPECT_EQ(fields.starter.primary_to_partner[0],2u);
    EXPECT_EQ(fields.starter.main_count,2u);EXPECT_EQ(fields.starter.node_count,20u);
}
}
