#include "Fixture.h"
#include <climits>
#include "lib_src/math/ScalarBits.h"
namespace crash::cases::vehicle_wall::native::test {
TEST(EnvelopeWallSource, ExplicitMaterialAndNativeHalfGapPreserveTheChosenContactPlane) {
    const auto geometry=detail::BuildGeometry(Box(),Decl(),Ids(),Units());
    EXPECT_EQ(geometry.native_half_gap,.5);
    EXPECT_EQ(geometry.reference_offset_m,.0005);
    EXPECT_EQ(geometry.component_primary_stiffness_native,200000.);
    EXPECT_EQ(geometry.reference_input.density,7860.);
    EXPECT_EQ(geometry.reference_input.projection_working_length_m,.001);
    EXPECT_EQ(geometry.reference_plane_m-geometry.reference_offset_m,geometry.placement.represented_wall_x_m);
    for(unsigned i=0;i<4;++i) {
        EXPECT_TRUE(tl::math::SameScalarBits(geometry.reference_native[i].x*.001,geometry.reference_m[i].x));
        EXPECT_GT(geometry.reference.nodal_mass[i],0);EXPECT_GT(geometry.reference.isotropic_inertia[i],0);
        EXPECT_EQ(geometry.reference_input.node_ids[i],Ids().nodes[i]);
        EXPECT_NE(geometry.display_feature_ids[i],Ids().nodes[i]);
    }
    auto bigger=Box();bigger[0].y=-1.5;bigger[1].y=1.5;
    const auto wide=detail::BuildGeometry(bigger,Decl(),Ids(),Units());
    EXPECT_GT(wide.reference.area,geometry.reference.area);
    EXPECT_GT(wide.wall_mass_kg,geometry.wall_mass_kg);
    EXPECT_EQ(wide.component_primary_stiffness_native,geometry.component_primary_stiffness_native);
}
TEST(EnvelopeWallSource, UnsupportedMaterialPlacementOrIdentityNeverBecomesAReference) {
    auto input=Decl();input.material.density_kg_m3=7.86;
    EXPECT_THROW(detail::BuildGeometry(Box(),input,Ids(),Units()),std::exception);
    input=Decl();input.profile=Profile::Unspecified;
    EXPECT_THROW(detail::BuildGeometry(Box(),input,Ids(),Units()),std::exception);
    input=Decl();input.leading_gap_m=-1;
    EXPECT_THROW(detail::BuildGeometry(Box(),input,Ids(),Units()),std::exception);
    auto ids=Ids();ids.nodes[2]=ids.nodes[1];
    EXPECT_THROW(detail::BuildGeometry(Box(),Decl(),ids,Units()),std::exception);
}
TEST(EnvelopeWallSource, FreshCombinedDomainPreservesVehicleBitsAndSeparatesMetricMassScope) {
    const tl::fea::NodalDomainNode original[]{{7,{-0.,-.75,0}},{2,{1,.75,1.5}}};
    tl::fea::NodalNodeDomain vehicle;
    ASSERT_TRUE(vehicle.Initialize({99,original,2}));
    const auto before=detail::Bounds(vehicle);
    const auto geometry=detail::BuildGeometry(before,Decl(),Ids(),Units());
    const auto combined=detail::BuildDomain(vehicle,Ids(),geometry,{});
    ASSERT_EQ(combined.domain.node_count(),6u);EXPECT_EQ(vehicle.node_count(),2u);
    EXPECT_FALSE(combined.domain.SharesStorage(vehicle));
    for(unsigned i=0;i<2;++i) {
        EXPECT_EQ(combined.domain.nodes()[i].source_id,original[i].source_id);
        EXPECT_TRUE(tl::math::SameScalarBits(combined.domain.nodes()[i].position.x,original[i].position.x));
        EXPECT_EQ(combined.fixed[i],0);EXPECT_EQ(combined.rotation[i],0);
    }
    for(unsigned i=2;i<6;++i){EXPECT_EQ(combined.fixed[i],7);EXPECT_EQ(combined.rotation[i],1);}
    EXPECT_EQ(detail::Bounds(vehicle)[1].x,before[1].x);
    EXPECT_GT(detail::Bounds(combined.domain)[1].x,before[1].x);
    auto bad=Ids();bad.nodes[0]=7;
    EXPECT_THROW(detail::BuildDomain(vehicle,bad,geometry,{}),std::exception);
    auto limits=Limits{};limits.domain_bytes=1;
    EXPECT_THROW(detail::BuildDomain(vehicle,Ids(),geometry,limits),std::exception);
    EXPECT_EQ(vehicle.node_count(),2u);
}
TEST(EnvelopeWallSource, CompleteNamespaceIncludesTransformedAndOmittedDefinitions) {
    NamespaceFixture fixture;
    const auto result=fixture.Build();
    EXPECT_EQ(result.first.maximum_declared,11001u);EXPECT_EQ(result.first.maximum_spring,102u);
    EXPECT_EQ(result.second.nodes[0],11002u);EXPECT_EQ(result.second.interface,11012u);
    EXPECT_TRUE(result.first.generated_node_after_complete_input);
    EXPECT_EQ(result.first.display_material_keyword,"*MAT_RIGID");
    EXPECT_EQ(result.first.display_density_native,7.86e-12);
    EXPECT_EQ(result.first.display_young_native,200000.);
    NamespaceFixture changed(20000);const auto other=changed.Build();
    EXPECT_NE(other.first.digest,result.first.digest);EXPECT_EQ(other.second.nodes[0],21002u);
    // Native SBACID is10 columns: an8-column read collapses these distinct IDs.
    NamespaceFixture sensors(10000,{},true);
    const auto sensor_namespace=sensors.Build();
    EXPECT_EQ(sensor_namespace.first.maximum_declared,5000002u);
    EXPECT_EQ(sensor_namespace.second.nodes[0],5000003u);
    EXPECT_EQ(sensor_namespace.first.definitions,result.first.definitions+2);
}
TEST(EnvelopeWallSource, NamespaceCollisionOverflowUnknownCardsAndCapsReject) {
    NamespaceFixture collision(0);
    EXPECT_THROW(collision.Build(),std::exception);
    NamespaceFixture overflow(std::uint64_t(INT_MAX)-1006);
    EXPECT_THROW(overflow.Build(),std::exception);
    NamespaceFixture unknown(10000,"*UNRESOLVED_NAMESPACE_ID");
    EXPECT_THROW(unknown.Build(),std::exception);
    NamespaceFixture valid;auto limits=Limits{};limits.namespace_entries=1;
    EXPECT_THROW(valid.Build(limits),std::exception);
    limits={};limits.namespace_bytes=1;
    EXPECT_THROW(valid.Build(limits),std::exception);
    EXPECT_NO_THROW(valid.Build());
}
} // namespace crash::cases::vehicle_wall::native::test
