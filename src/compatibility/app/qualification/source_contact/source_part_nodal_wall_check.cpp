#include "SourceNodalWallTest.h"
#include <iostream>

namespace crash::qualification::source_contact::nodal::test {
TEST_F(Check, All94IndependentSourceParentsKeepMassIdentityAndQuantifiedModelDifference) {
    cw::WallTessellation original; ASSERT_NO_FATAL_FAILURE(Geometry(original));
    const auto& geometry=*original.geometry(); cf::Harness integral(source);
    ASSERT_TRUE(integral.mass.Initialize(source));
    unsigned quads=0,triangles=0; double maximum_force_difference=0,maximum_energy_difference=0;
    for (unsigned p=0;p<ParentCount;++p) {
        const auto& binding=source.parents()[p]; SCOPED_TRACE(binding.source_id);
        sc::NodalWallWeights one; ASSERT_TRUE(prepared.ParentWeights(p,&one));
        ASSERT_EQ(one.parent_count(),1u); ASSERT_EQ(one.node_count(),binding.arity);
        EXPECT_EQ(one.parent(0).parent_element_id,binding.source_id); EXPECT_EQ(one.parent(0).feature_id,binding.source_id);
        EXPECT_EQ(one.parent(0).arity,binding.arity);
        for (unsigned l=0;l<binding.arity;++l) EXPECT_EQ(one.parent(0).nodes[l],binding.local_node_indices[l]);
        const double shift=cf::ParentShift(source,p,geometry.wall_x());
        const auto position=cf::Shift(source,shift),velocity=cf::Velocity(shift);
        cf::ParentForce continuous;
        ASSERT_TRUE(integral.EvaluateParent(p,source.coordinates(),position,velocity,geometry,&continuous))<<integral.diagnostic;
        sc::NodalWallResult result;
        ASSERT_EQ(sc::EvaluateNodalWallContact(one,cf::View(position),cf::View(velocity),prepared.mass().view(),
            {geometry.wall_x(),cf::Stiffness,cf::Cap,cf::ForceBudget,cf::EnergyBudget},31,&result).status,sc::NodalWallStatus::Ok);
        ASSERT_TRUE(result.valid); ASSERT_EQ(result.parent_count,1u); EXPECT_EQ(result.base_epoch,17u); EXPECT_EQ(result.attempt,31u);
        const auto& nodal=result.parents[0];
        EXPECT_LE(nodal.resultant.error,cf::ForceBudget); EXPECT_LE(nodal.potential.error,cf::EnergyBudget);
        EXPECT_GT(nodal.resultant.lower,0); EXPECT_GT(nodal.potential.lower,0);
        const long double df=static_cast<long double>(nodal.resultant.value)-continuous.resultant.value;
        const long double du=static_cast<long double>(nodal.potential.value)-continuous.potential.value;
        const long double ef=static_cast<long double>(nodal.resultant.error)+continuous.resultant.error;
        const long double eu=static_cast<long double>(nodal.potential.error)+continuous.potential.error;
        // Jensen is about resultant and potential, not ordering each nodal force.
        // These are two different discrete models; no integral-error promotion.
        EXPECT_GE(df+ef,0); EXPECT_GE(du+eu,0);
        maximum_force_difference=std::max(maximum_force_difference,static_cast<double>(df));
        maximum_energy_difference=std::max(maximum_energy_difference,static_cast<double>(du));
        for (unsigned n=0;n<result.node_count;++n) {
            const auto& node=result.nodes[n]; EXPECT_TRUE(node.row.valid); EXPECT_EQ(node.row.count,1u);
            EXPECT_EQ(node.row.nodes[0],node.node); EXPECT_GT(prepared.mass().mass[node.node],0);
            EXPECT_EQ(node.local_velocity_first_timestep,0);
        }
        RecordProperty("parent_"+std::to_string(binding.source_id)+"_force_model_difference_N",Number(static_cast<double>(df)));
        RecordProperty("parent_"+std::to_string(binding.source_id)+"_potential_model_difference_J",Number(static_cast<double>(du)));
        RecordProperty("parent_"+std::to_string(binding.source_id)+"_force_uncertainty_N",Number(static_cast<double>(ef)));
        RecordProperty("parent_"+std::to_string(binding.source_id)+"_potential_uncertainty_J",Number(static_cast<double>(eu)));
        if (binding.arity==4) ++quads; else ++triangles;
    }
    EXPECT_EQ(quads,88u); EXPECT_EQ(triangles,6u); EXPECT_EQ(prepared.weights().node_count(),117u);
    for (unsigned n=0;n<NodeCount;++n) {
        EXPECT_EQ(prepared.mass().mass[n],integral.mass.mass[n]);
        EXPECT_EQ(prepared.mass().inverse[n],integral.mass.inverse[n]);
        EXPECT_EQ(prepared.weights().node(n).node,n);
    }
    RecordProperty("experiments","94 separate parent-preserving rigid shifts; not one whole-part motion");
    RecordProperty("maximum_force_model_difference_N",Number(maximum_force_difference));
    RecordProperty("maximum_potential_model_difference_J",Number(maximum_energy_difference)); Metadata();
}

TEST_F(Check, HostSweepRejectsOutsideHoleFixedMotionAndBothEndpointCapsWithoutPublishing) {
    cw::WallTessellation original; ASSERT_NO_FATAL_FAILURE(Geometry(original)); const auto& wall_geometry=*original.geometry();
    Input input; ASSERT_NO_FATAL_FAILURE(Coherent(wall_geometry,input)); const auto initial=Bytes(input);
    const double shift=cf::WholeShift(source,wall_geometry.wall_x()); auto endpoint=cf::Shift(source,shift);
    auto base=source.coordinates(); const auto velocity=cf::Velocity(shift);
    endpoint[3*(NodeCount-1)+1]+=100;
    EXPECT_EQ(BuildInput(prepared,base,endpoint,velocity,prepared.mass(),wall_geometry,31,&input,diagnostic).status,Status::OutsideWall);
    Unchanged(input,initial); endpoint=cf::Shift(source,shift);
    base[3*(NodeCount-1)]=wall_geometry.wall_x()+2*cf::Cap;
    EXPECT_EQ(BuildInput(prepared,base,endpoint,velocity,prepared.mass(),wall_geometry,31,&input,diagnostic).status,Status::PreflightFailure);
    Unchanged(input,initial); base=source.coordinates(); endpoint[3*(NodeCount-1)]=wall_geometry.wall_x()+2*cf::Cap;
    EXPECT_EQ(BuildInput(prepared,base,endpoint,velocity,prepared.mass(),wall_geometry,31,&input,diagnostic).status,Status::PreflightFailure);
    Unchanged(input,initial);

    auto mass=prepared.mass(); mass.fixed[NodeCount-1]=1; mass.inverse[NodeCount-1]=0;
    const cf::Coordinates zero{}; base=source.coordinates(); endpoint=base;
    ASSERT_EQ(BuildInput(prepared,base,endpoint,zero,mass,wall_geometry,31,&input,diagnostic).status,Status::Ok)<<diagnostic;
    Result fixed; ASSERT_EQ(EvaluateHost(prepared,input,&fixed).status,Status::Ok);
    EXPECT_TRUE(fixed.contact.nodes[NodeCount-1].fixed); EXPECT_FALSE(fixed.contact.nodes[NodeCount-1].row.valid);
    EXPECT_EQ(fixed.contact.nodes[NodeCount-1].force.value,0); const auto accepted_fixed=Bytes(input);
    endpoint[3*(NodeCount-1)]+=.00001;
    EXPECT_EQ(BuildInput(prepared,base,endpoint,zero,mass,wall_geometry,31,&input,diagnostic).status,Status::PreflightFailure);
    Unchanged(input,accepted_fixed);
    endpoint=base; base[3*(NodeCount-1)]=endpoint[3*(NodeCount-1)]=wall_geometry.wall_x()+cf::Depth;
    EXPECT_EQ(BuildInput(prepared,base,endpoint,zero,mass,wall_geometry,31,&input,diagnostic).status,Status::PreflightFailure);
    Unchanged(input,accepted_fixed);

    sc::PlanarWallGeometry hole; sc::Vec3 center; const unsigned node=MaximumX();
    ASSERT_NO_FATAL_FAILURE(Hole(original,source.positions().at(node),hole,center));
    base=source.coordinates(); endpoint=cf::Shift(source,shift);
    const double dy=center.y-source.positions().at(node).y,dz=center.z-source.positions().at(node).z;
    for (unsigned n=0;n<NodeCount;++n) {
        base[3*n+1]+=dy; endpoint[3*n+1]+=dy; base[3*n+2]+=dz; endpoint[3*n+2]+=dz;
    }
    // First prove the same whole-part prescribed sweep fits the intact wall.
    ASSERT_EQ(BuildInput(prepared,base,endpoint,velocity,prepared.mass(),wall_geometry,31,&input,diagnostic).status,Status::Ok)<<diagnostic;
    const auto before_hole=Bytes(input);
    EXPECT_EQ(BuildInput(prepared,base,endpoint,velocity,prepared.mass(),hole,31,&input,diagnostic).status,Status::OutsideWall);
    Unchanged(input,before_hole); Metadata();
}
} // namespace crash::qualification::source_contact::nodal::test

int main(int argc,char** argv) {
    ::testing::InitGoogleTest(&argc,argv);
    if (argc!=3) { std::cerr<<"Usage: source_part_nodal_wall_check readiness.json wall.manifest.json [gtest options]\n"; return 2; }
    crash::qualification::source_contact::nodal::test::readiness_path=argv[1];
    crash::qualification::source_contact::nodal::test::wall_path=argv[2];
    return RUN_ALL_TESTS();
}
