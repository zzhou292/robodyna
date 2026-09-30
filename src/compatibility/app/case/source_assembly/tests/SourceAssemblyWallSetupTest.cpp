#include "SourceAssemblyWallSetupTestSupport.h"
#include <algorithm>
#include <limits>
#include <memory>

namespace crash::cases::source_assembly::test {
namespace sc=tlfea::contact;
TEST(SourceAssemblyWallSetup, EntireAssemblyWallAndSeparateNativeAggregateEnergyAreCertified) {
    const auto& assembly=WallAssembly();WallInput wall;const auto settings=WallSettings();SourceAssemblyWallSetup setup;
    const auto report=setup.Initialize(assembly,wall.canonical,wall.bytes,settings);ASSERT_TRUE(report)<<report.message;
    const auto& c=*setup.certificate();const auto& b=setup.bindings()->shells();const auto& weights=*setup.source_geometry()->weights();
    ASSERT_EQ(weights.parent_count(),915u);ASSERT_EQ(weights.node_count(),1030u);
    EXPECT_EQ(setup.source_geometry()->binding()->inventory(),assembly.shells().inventory());
    EXPECT_EQ(c.initial_kinetic.groups.size(),6u);EXPECT_EQ(c.initial_kinetic.member_nodes,76u);
    EXPECT_EQ(c.initial_kinetic.ordinary_nodes,954u);EXPECT_EQ(c.initial_kinetic.generated_primaries,6u);
    EXPECT_EQ(c.penalty.initial_kinetic.metric,wall_penalty::InitialKineticMetric::NativeNodesWithAggregateGroups);
    SameBits(c.penalty.initial_kinetic.enclosure.value,c.initial_kinetic.with_aggregate_groups.value);
    long double native=0,aggregate=0,primary=0;double nominal=0;std::vector<bool> member(b.node_count());
    const long double speed=settings.initial_velocity[0];const auto* groups=assembly.rigid_groups();
    for(unsigned g=0;g<groups->group_count();++g) {
        const auto& properties=groups->groups()[g];const auto& measured=c.initial_kinetic.groups[g];long double member_energy=0;
        EXPECT_EQ(measured.source_group_id,properties.source_group_id);EXPECT_EQ(measured.source_node_set_id,properties.source_node_set_id);
        for(unsigned m=0;m<properties.member_count;++m) {
            const auto& input=groups->members()[properties.member_offset+m];EXPECT_FALSE(member[input.global_node]);
            member[input.global_node]=true;member_energy+=.5L*input.mass_kg*speed*speed;
        }
        Encloses(measured.native_members,member_energy);
        const long double group_energy=.5L*properties.total_mass_kg*speed*speed;
        Encloses(measured.aggregate,group_energy);aggregate+=group_energy;
        // total_mass already includes primary regularization. Record that
        // native ledger separately and never add it a second time to energy.
        SameBits(measured.generated_primary_mass_kg,properties.regularization.primary_mass_kg);
        primary+=properties.regularization.primary_mass_kg;
    }
    for(unsigned n=0;n<b.node_count();++n) {
        const double mass=b.nodes()[n].native.mass;native+=.5L*mass*speed*speed;
        nominal+=.5*mass*(settings.initial_velocity[0]*settings.initial_velocity[0]);
        if(!member[n])aggregate+=.5L*mass*speed*speed;
        EXPECT_EQ(weights.node(n).node,n);EXPECT_GE(weights.node(n).area.lower,settings.penalty.area_floor);
        const auto x=setup.source_geometry()->positions().at(n);const auto reference=b.nodes()[n].position;
        SameBits(x.x,reference.x);SameBits(x.y,reference.y);SameBits(x.z,reference.z);
    }
    Encloses(c.initial_kinetic.native_nodes,native);SameBits(c.initial_kinetic.native_nodes.value,nominal);
    Encloses(c.initial_kinetic.with_aggregate_groups,aggregate);Encloses(c.initial_kinetic.generated_primary_mass_kg,primary);
    EXPECT_GE(c.penalty.design_potential_lower,c.penalty.kinetic_budget_upper);
    const long double potential=.5L*c.penalty.stiffness_per_area*settings.penalty.area_floor*
        settings.penalty.design_penetration*settings.penalty.design_penetration;
    EXPECT_LE(static_cast<long double>(c.penalty.design_potential_lower),potential);
    const auto placed=setup.placed_wall()->view();const double x=setup.placed_wall()->geometry()->wall_x();
    const auto bounds=setup.source_geometry()->reference_bounds();EXPECT_TRUE(c.coverage.covered);
    EXPECT_EQ(c.coverage.physical.minimum.x,x);EXPECT_EQ(c.coverage.physical.maximum.x,x);
    EXPECT_GT(c.leading_gap.lower,0);EXPECT_LE(c.leading_gap.lower,static_cast<long double>(x)-bounds[1].x);
    EXPECT_GE(c.leading_gap.upper,static_cast<long double>(x)-bounds[1].x);
    ASSERT_EQ(placed.vertex_count,62u);ASSERT_EQ(placed.triangle_count,100u);
    for(unsigned i=0;i<placed.vertex_count;++i) {
        const auto& original=wall.canonical.vertices()[i];const auto& current=placed.vertices[i];
        SameBits(current.position.y,original.position_m[1]);SameBits(current.position.z,original.position_m[2]);
        EXPECT_EQ(current.source_node_id,original.source_node_id);EXPECT_EQ(current.assembled_source_node_id,original.assembled_source_node_id);
    }
    const auto& boundary=setup.bindings()->source().data().boundary;
    EXPECT_EQ(boundary.policy,"released_external_connections");EXPECT_EQ(boundary.nodal_rigid_ids.size(),4u);
    EXPECT_EQ(boundary.spotweld_ids.size(),13u);EXPECT_FALSE(boundary.tied_scopes.empty());
    RecordProperty("native_initial_kinetic_j",Number(c.initial_kinetic.native_nodes.value));
    RecordProperty("aggregate_initial_kinetic_j",Number(c.initial_kinetic.with_aggregate_groups.value));
    RecordProperty("minimum_area_lower_m2",Number(c.penalty.minimum_nodal_area_lower));
    RecordProperty("stiffness_per_area",Number(c.penalty.stiffness_per_area));
}
TEST(SourceAssemblyWallSetup, ExplicitBoundaryMalformedLimitsAndLateAreaCoverageFailurePreservePublication) {
    const auto& assembly=WallAssembly();WallInput wall;const auto settings=WallSettings();SourceAssemblyWallSetup setup;
    auto bad=settings;bad.boundary=SourceAssemblyWallBoundary::Unspecified;
    EXPECT_EQ(setup.Initialize(assembly,wall.canonical,wall.bytes,bad).status,SourceAssemblyWallStatus::InvalidInput);
    bad=settings;bad.initial_velocity[2]=std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(setup.Initialize(assembly,wall.canonical,wall.bytes,bad).status,SourceAssemblyWallStatus::InvalidInput);
    for(unsigned variant=0;variant<7;++variant) {
        SourceAssemblyWallLimits limits;
        if(variant==0)limits.max_startup_bytes=1;
        if(variant==1)limits.max_startup_bytes=SIZE_MAX;
        if(variant==2)limits.max_wall_manifest_bytes=wall.bytes.size()-1;
        if(variant==3)limits.geometry.max_startup_bytes=1;
        if(variant==4)limits.geometry.weights.max_nodes=1029;
        if(variant==5)limits.geometry.weights.max_parents=914;
        if(variant==6)limits.geometry.weights.max_owned_bytes=1;
        EXPECT_EQ(setup.Initialize(assembly,wall.canonical,wall.bytes,settings,limits).status,SourceAssemblyWallStatus::ResourceLimit);
        EXPECT_FALSE(setup.initialized());EXPECT_EQ(setup.certificate(),nullptr);
    }
    bad=settings;bad.penalty.area_floor=1;
    EXPECT_EQ(setup.Initialize(assembly,wall.canonical,wall.bytes,bad).status,SourceAssemblyWallStatus::CertificateFailure);
    bad=settings;bad.motion_margin=3;
    EXPECT_EQ(setup.Initialize(assembly,wall.canonical,wall.bytes,bad).status,SourceAssemblyWallStatus::GeometryFailure);
    EXPECT_FALSE(setup.initialized());ASSERT_TRUE(setup.Initialize(assembly,wall.canonical,wall.bytes,settings));
    const auto* certificate=setup.certificate();const auto stiffness=certificate->penalty.stiffness_per_area;
    EXPECT_EQ(setup.Initialize(assembly,wall.canonical,wall.bytes,bad).status,SourceAssemblyWallStatus::AlreadyInitialized);
    EXPECT_EQ(setup.certificate(),certificate);SameBits(setup.certificate()->penalty.stiffness_per_area,stiffness);
}
TEST(SourceAssemblyWallSetup, ImmutableCopiesOwnCompleteSourceAndPlacedWallAfterInputsExpire) {
    auto prepared=[] {
        const auto source=SourceAssemblyBindings::Prepare(Load(),Options());WallInput wall;SourceAssemblyWallSetup setup;
        const auto result=setup.Initialize(source,wall.canonical,wall.bytes,WallSettings());
        EXPECT_TRUE(result)<<result.message;return setup;
    }();
    ASSERT_TRUE(prepared.initialized());SourceAssemblyWallSetup copied(prepared),moved(std::move(prepared));
    EXPECT_TRUE(prepared.initialized());EXPECT_EQ(copied.certificate(),moved.certificate());
    EXPECT_EQ(copied.source_geometry()->weights()->node(1029).node,1029u);
    EXPECT_EQ(copied.bindings()->source().data().parents.size(),915u);
    EXPECT_EQ(copied.bindings()->source().data().authenticated_bytes,WallAssembly().source().data().authenticated_bytes);
    EXPECT_EQ(copied.placed_wall()->view().triangle_count,100u);
}
} // namespace crash::cases::source_assembly::test
