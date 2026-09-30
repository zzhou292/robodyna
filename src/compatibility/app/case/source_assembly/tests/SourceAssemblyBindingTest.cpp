#include "SourceAssemblyBindingTestSupport.h"
#include <set>

namespace crash::cases::source_assembly::test {
TEST(SourceAssemblyBindings, AllActualNativeReferencesAndSourceMaterialsPrepareTogether) {
    const auto prepared=SourceAssemblyBindings::Prepare(Load(),Options());
    const auto& data=prepared.source().data(); const auto& shells=prepared.shells(); const auto& materials=prepared.materials();
    EXPECT_EQ(shells.qeph_count(),804u); EXPECT_EQ(shells.t3_count(),111u); EXPECT_EQ(shells.node_count(),1030u);
    EXPECT_EQ(shells.inventory().words().size(),24154u); EXPECT_TRUE(materials.Matches(shells));
    EXPECT_EQ(materials.parent_count(),915u); EXPECT_EQ(materials.material_count(),6u); EXPECT_EQ(materials.curve_point_count(),63u);
    for(const auto& parent:data.parents) {
        const bool q=parent.family==source::ShellFamily::Qeph;
        ASSERT_TRUE(q?shells.qeph_reference(parent.family_index).prepared:shells.t3_reference(parent.family_index).prepared);
        EXPECT_EQ(q?shells.qeph_source_id(parent.family_index):shells.t3_source_id(parent.family_index),parent.source_id);
        fe::sections::PointParameters parameters;
        ASSERT_TRUE(materials.Parameters(q?fe::ShellBindingFamily::Qeph:fe::ShellBindingFamily::T3,parent.family_index,&parameters));
        const auto& material=data.materials[parent.material_index]; const auto& curve=data.curves[parent.curve_index];
        SameBits(parameters.young_pa,material.young_pa); SameBits(parameters.density_kg_m3,material.density_kg_m3);
        ASSERT_EQ(parameters.curve.count,curve.plastic_strain.size());
        SameBits(parameters.curve.yield_stress_pa[parameters.curve.count-1],curve.stress_pa.back());
    }
    EXPECT_EQ(prepared.source_instance_id(),Options().source_instance_id);
    EXPECT_EQ(data.boundary.policy,"released_external_connections");
    EXPECT_EQ(data.boundary.nodal_rigid_ids.size(),4u); EXPECT_EQ(data.boundary.spotweld_ids.size(),13u);
    EXPECT_EQ(data.boundary.external_node_ids.size(),37u); EXPECT_EQ(data.boundary.tied_scopes.size(),1u);
    RecordProperty("native_mass_kg",Number(shells.totals().mass));
    RecordProperty("native_total_inertia_kg_m2",Number(shells.totals().isotropic_inertia));
    RecordProperty("physical_inertia_kg_m2",Number(shells.totals().physical_inertia));
    RecordProperty("added_inertia_kg_m2",Number(shells.totals().added_inertia));
}
TEST(SourceAssemblyBindings, SourcePartLedgersAgreeWithIndependentDisjointNodeReduction) {
    const auto prepared=SourceAssemblyBindings::Prepare(Load(),Options()); const auto& data=prepared.source().data();
    ASSERT_EQ(prepared.part_mass_ledger().size(),6u); std::set<std::size_t> all_nodes;
    long double total[4]{};
    for(std::size_t i=0;i<data.parts.size();++i) {
        const auto& part=data.parts[i]; const auto& ledger=prepared.part_mass_ledger()[i];
        EXPECT_EQ(ledger.source_part_id,part.id); EXPECT_EQ(ledger.parent_count,part.parent_count);
        EXPECT_EQ(ledger.qeph_count+ledger.t3_count,part.parent_count);
        // The selected six parts have disjoint nodes. Their nodal union provides
        // an independent route to the per-parent diagnostic ledger, with no new mass formula.
        long double expected[4]{};
        for(const auto index:part.nodes) {
            ASSERT_TRUE(all_nodes.insert(index).second); const auto& m=prepared.shells().nodes()[index].native;
            const double values[]{m.mass,m.isotropic_inertia,m.physical_inertia,m.added_inertia};
            for(unsigned channel=0;channel<4;++channel) expected[channel]+=values[channel];
        }
        const auto& m=ledger.native; const double actual[]{m.mass,m.isotropic_inertia,m.physical_inertia,m.added_inertia};
        for(unsigned channel=0;channel<4;++channel) { Near(actual[channel],expected[channel]); total[channel]+=actual[channel]; }
        const auto prefix="part_"+std::to_string(part.id);
        RecordProperty(prefix+"_mass_kg",Number(m.mass)); RecordProperty(prefix+"_native_total_inertia_kg_m2",Number(m.isotropic_inertia));
        RecordProperty(prefix+"_physical_inertia_kg_m2",Number(m.physical_inertia)); RecordProperty(prefix+"_added_inertia_kg_m2",Number(m.added_inertia));
    }
    EXPECT_EQ(all_nodes.size(),1030u); const auto& global=prepared.shells().totals();
    Near(global.mass,total[0]); Near(global.isotropic_inertia,total[1]);
    Near(global.physical_inertia,total[2]); Near(global.added_inertia,total[3]);
}
TEST(SourceAssemblyBindings, LifetimeAndIndependentStageFailuresNeverReplacePublishedComposition) {
    auto source_model=std::make_unique<source::SourceAssembly>(Load());
    const auto held=SourceAssemblyBindings::Prepare(*source_model,Options());
    source_model.reset(); SourceAssemblyBindings copied(held),moved(std::move(copied));
    EXPECT_EQ(&held.shells(),&moved.shells()); EXPECT_EQ(&copied.source().data(),&held.source().data());
    const auto* source_nodes=held.source().data().nodes.data(); const auto identity=held.shells().inventory();
    for(unsigned failure=0;failure<6;++failure) {
        auto options=Options();
        if(failure==0) options.source_instance_id=0;
        if(failure==1) options.shell_limits.max_parents=914;
        if(failure==2) options.shell_limits.max_owned_bytes=held.shells().host_bytes()-1;
        if(failure==3) options.material_limits.max_owned_bytes=held.materials().host_bytes()-1;
        if(failure==4) options.rigid_limits.max_host_bytes=held.rigid_groups()->startup_payload_bytes()-1;
        if(failure==5) options.rigid_limits.max_members=75;
        const SourceAssemblyBindingStage expected[]{SourceAssemblyBindingStage::Input,SourceAssemblyBindingStage::Shells,
            SourceAssemblyBindingStage::Shells,SourceAssemblyBindingStage::Materials,
            SourceAssemblyBindingStage::RigidGroups,SourceAssemblyBindingStage::RigidGroups};
        try { (void)SourceAssemblyBindings::Prepare(held.source(),options); FAIL()<<"Expected stage admission rejection"; }
        catch(const SourceAssemblyBindingError& error) { EXPECT_EQ(error.stage,expected[failure]); }
        EXPECT_EQ(held.source().data().nodes.data(),source_nodes); EXPECT_EQ(held.shells().inventory(),identity);
        EXPECT_TRUE(held.materials().Matches(held.shells()));
    }
}
TEST(SourceAssemblyBindings, NativeFailureReportsExactSourceIdentityWithoutChangingGeometry) {
    const auto original=Load(); const auto& data=original.data();
    const auto expected=std::find_if(data.parents.begin(),data.parents.end(),[](const auto& parent) {
        return parent.part_id==2000260&&parent.family==source::ShellFamily::Qeph;
    });
    ASSERT_NE(expected,data.parents.end());
    source::test::Scratch scratch;
    // A separate explicitly identified corruption fixture exercises native
    // reference admission. The frozen source artifact is never rewritten.
    const auto bytes=source::test::Alter([](auto& document) {
        for(auto& node:document["geometry"]["parts"][5]["nodes"].GetArray())
            for(auto& coordinate:node["position_m"].GetArray()) coordinate.SetDouble(0.);
    });
    const auto invalid=source::SourceAssembly::Read(scratch.Write(bytes),source::test::ExplicitTestIdentity(bytes));
    try { (void)SourceAssemblyBindings::Prepare(invalid,Options()); FAIL()<<"Invalid source reference unexpectedly admitted"; }
    catch(const SourceAssemblyBindingError& error) {
        EXPECT_EQ(error.stage,SourceAssemblyBindingStage::Shells); EXPECT_EQ(error.status,unsigned(fe::ShellBindingStatus::InvalidQephReference));
        EXPECT_EQ(error.source_element_id,expected->source_id); EXPECT_EQ(error.source_part_id,expected->part_id);
        EXPECT_NE(std::string(error.what()).find("family=QEPH"),std::string::npos);
        EXPECT_NE(std::string(error.what()).find("native_status="),std::string::npos);
    }
    EXPECT_EQ(original.data().identity.sha256,source::PinnedYarisSixPartInventory().sha256);
}
} // namespace crash::cases::source_assembly::test
