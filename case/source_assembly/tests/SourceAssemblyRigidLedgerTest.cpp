#include "SourceAssemblyBindingTestSupport.h"
#include <set>

namespace crash::cases::source_assembly::test {
TEST(SourceAssemblyBindings, SixOriginalRigidGroupsUseOnlyThe76ExactNativeMembers) {
    const auto prepared=SourceAssemblyBindings::Prepare(Load(),Options()); const auto& data=prepared.source().data();
    const auto* rigid=prepared.rigid_groups(); ASSERT_NE(rigid,nullptr);
    EXPECT_TRUE(rigid->prepared()); EXPECT_EQ(rigid->group_count(),6u); EXPECT_EQ(rigid->member_count(),76u);
    EXPECT_EQ(rigid->global_node_count(),1030u); EXPECT_EQ(rigid->source_instance_id(),prepared.source_instance_id());
    SameBits(rigid->source_units().mass_to_kg,data.units.mass_to_kg);
    SameBits(rigid->source_units().length_to_m,data.units.length_to_m);
    std::set<std::size_t> selected; std::size_t active=0;
    for(const auto& source_group:data.nodal_rigid_groups) {
        if(!source_group.internal) continue;
        const auto& group=rigid->groups()[active++];
        EXPECT_EQ(group.source_group_id,source_group.id); EXPECT_EQ(group.source_node_set_id,source_group.node_set_id);
        ASSERT_EQ(group.member_count,source_group.members.size());
        for(std::size_t local=0;local<group.member_count;++local) {
            const auto& member=rigid->members()[group.member_offset+local];
            EXPECT_EQ(member.source_node_id,source_group.members[local]);
            EXPECT_EQ(member.global_node,source_group.selected_global_nodes[local]);
            ASSERT_TRUE(selected.insert(member.global_node).second);
            const auto& native=prepared.shells().nodes()[member.global_node];
            EXPECT_EQ(member.source_node_id,native.source_id);
            SameBits(member.position.x,native.position.x); SameBits(member.position.y,native.position.y); SameBits(member.position.z,native.position.z);
            SameBits(member.mass_kg,native.native.mass); SameBits(member.total_inertia_kg_m2,native.native.isotropic_inertia);
            SameBits(member.physical_inertia_kg_m2,native.native.physical_inertia); SameBits(member.added_inertia_kg_m2,native.native.added_inertia);
        }
    }
    EXPECT_EQ(active,6u); EXPECT_EQ(selected.size(),76u);
    // Generated primary regularizers are ledger channels, not appended source nodes.
    EXPECT_EQ(prepared.shells().node_count(),data.nodes.size());
    EXPECT_EQ(data.boundary.nodal_rigid_ids,(std::vector<source::SourceId>{2200054,2200055,2200921,2200946}));
}
TEST(SourceAssemblyBindings, ActualGroupMassTensorAndRegularizationChannelsAreAccounted) {
    const auto prepared=SourceAssemblyBindings::Prepare(Load(),Options());
    const auto& source_units=prepared.source().data().units; const auto* model=prepared.rigid_groups(); ASSERT_NE(model,nullptr);
    const double primary_mass=1e-20*source_units.mass_to_kg;
    const double primary_j=primary_mass*source_units.length_to_m*source_units.length_to_m;
    for(std::size_t index=0;index<model->group_count();++index) {
        const auto& group=model->groups()[index]; const auto* members=model->members()+group.member_offset;
        long double mass=0,j=0,physical=0,added=0,geometric[3]{},moment[3]{};
        for(std::size_t local=0;local<group.member_count;++local) {
            const auto& m=members[local]; const double x[]{m.position.x,m.position.y,m.position.z};
            mass+=m.mass_kg; j+=m.total_inertia_kg_m2; physical+=m.physical_inertia_kg_m2; added+=m.added_inertia_kg_m2;
            for(unsigned axis=0;axis<3;++axis) { geometric[axis]+=x[axis]; moment[axis]+=static_cast<long double>(m.mass_kg)*x[axis]; }
        }
        Near(group.structural_mass_kg,mass); Near(group.native_total_inertia_sum,j);
        Near(group.physical_inertia_sum,physical); Near(group.added_inertia_sum,added);
        SameBits(group.regularization.primary_mass_kg,primary_mass); SameBits(group.regularization.primary_isotropic_inertia_kg_m2,primary_j);
        Near(group.total_mass_kg,mass+primary_mass);
        const double center[]{group.center.x,group.center.y,group.center.z};
        const double generated[]{group.generated_primary_position.x,group.generated_primary_position.y,group.generated_primary_position.z};
        for(unsigned axis=0;axis<3;++axis) {
            geometric[axis]/=group.member_count;
            Near(generated[axis],geometric[axis]);
            Near(center[axis],(moment[axis]+primary_mass*geometric[axis])/(mass+primary_mass));
        }
        // Long-double parallel-axis identity checks the whole source group, including
        // the independently retained native isotropic J and generated primary ledger.
        long double tensor[9]{};
        const auto append=[&](const double* x,double m,double isotropic) {
            long double r[3]; for(unsigned a=0;a<3;++a) r[a]=static_cast<long double>(x[a])-center[a];
            const auto squared=r[0]*r[0]+r[1]*r[1]+r[2]*r[2];
            for(unsigned a=0;a<3;++a) for(unsigned b=0;b<3;++b)
                tensor[3*a+b]+=(a==b?isotropic+static_cast<long double>(m)*squared:0)-m*r[a]*r[b];
        };
        append(generated,primary_mass,primary_j);
        for(std::size_t local=0;local<group.member_count;++local) {
            const auto& m=members[local]; const double x[]{m.position.x,m.position.y,m.position.z}; append(x,m.mass_kg,m.total_inertia_kg_m2);
        }
        long double scale=0; for(auto value:tensor) scale=std::max(scale,std::abs(value));
        for(unsigned i=0;i<9;++i) {
            EXPECT_LE(std::abs(static_cast<long double>(group.raw_tensor.v[i])-tensor[i]),2e-11L*scale);
            const auto effective=static_cast<long double>(group.raw_tensor.v[i])+group.regularization.tensor_added.v[i];
            EXPECT_LE(std::abs(static_cast<long double>(group.effective_tensor.v[i])-effective),2e-11L*scale);
        }
        const auto prefix="group_"+std::to_string(group.source_group_id);
        RecordProperty(prefix+"_structural_mass_kg",Number(group.structural_mass_kg));
        RecordProperty(prefix+"_native_total_inertia_kg_m2",Number(group.native_total_inertia_sum));
        RecordProperty(prefix+"_physical_inertia_kg_m2",Number(group.physical_inertia_sum));
        RecordProperty(prefix+"_added_inertia_kg_m2",Number(group.added_inertia_sum));
        RecordProperty(prefix+"_primary_mass_kg",Number(group.regularization.primary_mass_kg));
        RecordProperty(prefix+"_primary_inertia_kg_m2",Number(group.regularization.primary_isotropic_inertia_kg_m2));
        const auto correction=group.regularization.principal_inertia_added;
        RecordProperty(prefix+"_principal_inertia_added_x",Number(correction.x));
        RecordProperty(prefix+"_principal_inertia_added_y",Number(correction.y));
        RecordProperty(prefix+"_principal_inertia_added_z",Number(correction.z));
        RecordProperty(prefix+"_principal_threshold_reached",group.regularization.principal_threshold_reached?"true":"false");
        RecordProperty(prefix+"_principal_correction",group.regularization.principal_inertia_changed?"true":"false");
    }
}
} // namespace crash::cases::source_assembly::test
