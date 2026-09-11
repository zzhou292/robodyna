#include "RigidSourceSupport.h"
#include "lib_utest/qualification/nodal_rigid_group/assembly/topology_fixture/YarisRigidTopologyFixture.h"
namespace crash::modelio::vehicle::test {
TEST(VehicleRigidSource, OriginalCoverageRootsAndChainedParametersRemainExact) {
    const auto& base=MidlayerResolution();const auto& current=RigidResolution();
    EXPECT_EQ(current.parents().data(),base.parents().data());
    EXPECT_EQ(&current.source().canonical().data(),&base.source().canonical().data());
    EXPECT_EQ(&current.failure_curves(),&base.failure_curves());
    EXPECT_EQ(current.counts().rigid_parts,22);EXPECT_EQ(current.counts().rigid_shells,5102);
    EXPECT_EQ(current.counts().unresolved_shells,0);EXPECT_EQ(current.parents().size(),349645);
    EXPECT_FALSE(current.resolution_key()==base.resolution_key());
    ASSERT_NE(current.rigid_source(),nullptr);const auto& rigid=*current.rigid_source();const auto& t=rigid.topology();
    ASSERT_EQ(t.part_count(),22);ASSERT_EQ(t.root_count(),20);EXPECT_EQ(t.member_count(),5452);
    EXPECT_EQ(t.other_rigid_member_count(),7539);
    namespace fixture=yaris_rigid_topology_fixture;
    std::vector<std::uint64_t> union_nodes(t.root_members(),t.root_members()+t.member_count());
    std::sort(union_nodes.begin(),union_nodes.end());
    EXPECT_EQ(union_nodes,std::vector<std::uint64_t>(std::begin(fixture::Expected),std::end(fixture::Expected)));
    std::size_t primaries=0;
    for(std::size_t root=0;root<t.root_count();++root) {
        EXPECT_EQ(t.parts()[t.roots()[root].part_index].source_part_id,fixture::RootParts[root]);
        EXPECT_EQ(t.roots()[root].member_count,fixture::RootCounts[root]);
        primaries+=t.roots()[root].original_primary_count;
        const auto& r=t.roots()[root];
        for(std::size_t n=0;n<r.member_count;++n)
            EXPECT_EQ(rigid.root_for_node(t.root_members()[r.member_offset+n]),root);
    }
    EXPECT_EQ(primaries,22);
    EXPECT_EQ(rigid.root_for_node(0),SIZE_MAX);
    EXPECT_EQ(rigid.root_for_node(t.other_rigid_members()[0]),SIZE_MAX);
    std::size_t rigid_rows=0,prior=0;
    NativeFormulationCounts count;
    for(std::size_t e=0;e<current.parents().size();++e) {
        const auto p=current.parents()[e].part_index;
        ASSERT_NE(current.native_material(p),nullptr);ASSERT_NE(current.native_parent(e),nullptr);
        const auto* map=current.native_mapping(e);ASSERT_NE(map,nullptr);
        auto& n=map->family==tl::fea::ShellBindingFamily::Qeph?count.qeph:
                (map->family==tl::fea::ShellBindingFamily::T3?count.t3:count.qbat);
        EXPECT_EQ(map->family_index,n++);
        if(base.material(p)) {
            ++prior;EXPECT_EQ(current.material(p),base.material(p));EXPECT_EQ(current.section(p),base.section(p));
            EXPECT_EQ(current.native_material(p),base.native_material(p));
            EXPECT_EQ(current.section_formulation(p),base.section_formulation(p));
            EXPECT_EQ(current.native_parent(e)->policy,base.native_parent(e)->policy);
            EXPECT_EQ(current.role(p),SourceShellRole::ConstitutiveShell);
        } else {
            ++rigid_rows;EXPECT_EQ(current.role(p),SourceShellRole::OriginalRigidPart);
            EXPECT_LT(current.rigid_root_index(p),20);
            EXPECT_EQ(current.material(p)->source.keyword,"*MAT_RIGID");
            EXPECT_EQ(current.native_material(p)->law,tl::fea::ShellSectionLaw::LayeredLaw1Nip3);
            EXPECT_EQ(current.native_parent(e)->policy,tl::fea::ShellFailurePolicy::None);
            EXPECT_EQ(current.section_formulation(p),tl::fea::ShellSectionFormulation::LayeredNip3);
        }
    }
    EXPECT_EQ(prior,344543);EXPECT_EQ(rigid_rows,5102);EXPECT_EQ(count.qbat,4250);
    EXPECT_EQ(current.native_counts().qeph,count.qeph);EXPECT_EQ(current.native_counts().t3,count.t3);
    EXPECT_EQ(current.native_material(current.parts().size()),nullptr);
    EXPECT_EQ(current.rigid_root_index(current.parts().size()),SIZE_MAX);
    RecordProperty("rigid_profile_forecast_bytes",std::to_string(current.startup_budget_bytes()));
}
TEST(VehicleRigidSource, BadTailAndCapacityFailWithoutChangingBaseAndExactRetry) {
    const auto& base=MidlayerResolution();constexpr auto profile=ResolutionProfile::OriginalRigidPartsV1;
    const auto forecast=VehicleSectionResolution::ForecastOriginalRigidParts(base,profile);
    auto limits=ResolutionLimits{};limits.host_bytes=forecast-1;
    EXPECT_THROW(VehicleSectionResolution::ResolveOriginalRigidParts(base,OriginalMember(),profile,limits),std::runtime_error);
    EXPECT_THROW(VehicleSectionResolution::ResolveOriginalRigidParts(GlassResolution(),OriginalMember(),profile),std::runtime_error);
    EXPECT_THROW(VehicleSectionResolution::ResolveOriginalRigidParts(base,OriginalMember(),ResolutionProfile::Artifact),std::runtime_error);
    auto bad=OriginalMember();bad.back()^=1;
    EXPECT_THROW(VehicleSectionResolution::ResolveOriginalRigidParts(base,bad,profile),std::runtime_error);
    bad.clear();bad.shrink_to_fit();
    limits.host_bytes=forecast;
    const auto current=VehicleSectionResolution::ResolveOriginalRigidParts(base,OriginalMember(),profile,limits);
    EXPECT_EQ(current.startup_budget_bytes(),forecast);EXPECT_EQ(base.counts().unresolved_shells,5102);
    EXPECT_EQ(base.rigid_source(),nullptr);
    EXPECT_THROW(VehicleSectionResolution::ResolveOriginalRigidParts(current,OriginalMember(),profile),std::runtime_error);
}
}
