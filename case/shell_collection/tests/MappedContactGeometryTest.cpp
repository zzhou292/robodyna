#include "case/shell_collection/ShellCollectionContactGeometry.h"
#include "lib_utest/qualification/qbat_binding/Fixture.h"
#include <algorithm>

namespace crash::cases::test {
namespace fe=tl::fea;
namespace sc=tlfea::contact;
namespace {
fe::ShellNodeMap MappedLayers() {
    qbat_binding_test::Fixture source;fe::ShellBatchBinding shells;
    EXPECT_EQ(shells.InitializeFormulations(source.Input()).status,fe::ShellBindingStatus::Success);
    std::vector<fe::NodalDomainNode> nodes{{900,{100,200,300}}};
    for(std::size_t i=shells.node_count();i-->0;)
        nodes.push_back({shells.nodes()[i].source_id,shells.nodes()[i].position});
    nodes.push_back({901,shells.nodes()[0].position}); // Coincident different NID is still not a shell vertex.
    fe::NodalNodeDomain domain;EXPECT_TRUE(domain.Initialize({42,nodes.data(),nodes.size()}));
    fe::ShellNodeMap map;EXPECT_TRUE(map.Initialize(shells,domain));return map;
}
void Check(const ShellCollectionContactGeometry& geometry,const fe::ShellNodeMap* map) {
    const auto& binding=*geometry.binding();const auto& weights=*geometry.weights();
    ASSERT_EQ(weights.parent_count(),4u);ASSERT_EQ(weights.node_count(),5u);
    EXPECT_EQ(weights.global_node_count(),map?7u:5u);
    EXPECT_EQ(geometry.positions().node_count,weights.global_node_count());
    std::size_t count[3]{};
    for(std::size_t p=0;p<weights.parent_count();++p) {
        const auto& parent=weights.parent(p);const auto* key=geometry.parent_from_weight(p);
        ASSERT_NE(key,nullptr);EXPECT_EQ(parent.parent_element_id,key->source_id);
        const bool tri=key->family==fe::ShellBindingFamily::T3;
        const bool qb=key->family==fe::ShellBindingFamily::Qbat;
        ++count[tri?1:qb?2:0];EXPECT_EQ(parent.arity,tri?3u:4u);
        for(unsigned l=0;l<parent.arity;++l) {
            const auto local=tri?binding.t3_nodes(key->family_index)[l]:
                qb?binding.qbat_nodes(key->family_index)[l]:binding.qeph_nodes(key->family_index)[l];
            EXPECT_EQ(parent.nodes[l],map?map->owner_index(local):local);
        }
        const double expected=tri?.0001:.0008;
        EXPECT_LE(parent.area.lower,expected);EXPECT_GE(parent.area.upper,expected);
    }
    EXPECT_EQ(count[0],2u);EXPECT_EQ(count[1],1u);EXPECT_EQ(count[2],1u);
    EXPECT_LE(weights.total_area().lower,.0025);EXPECT_GE(weights.total_area().upper,.0025);
    EXPECT_EQ(geometry.reference_bounds()[0].x,0);EXPECT_EQ(geometry.reference_bounds()[1].x,.05);
    EXPECT_EQ(geometry.reference_bounds()[1].y,.02);EXPECT_EQ(geometry.reference_bounds()[1].z,0);
}
}
TEST(MappedContactGeometry, AllThreePhysicalLayersContributeTheirOwnNativeArea) {
    const auto map=MappedLayers();ShellCollectionContactGeometry geometry;
    const auto report=geometry.Initialize(*map.shells());ASSERT_TRUE(report)<<report.message;
    ASSERT_NO_FATAL_FAILURE(Check(geometry,nullptr));EXPECT_EQ(geometry.mapping(),nullptr);
}
TEST(MappedContactGeometry, ScrambledDomainRetainsAllFamiliesAndExcludesExtraCoincidentNodes) {
    ShellCollectionContactGeometry geometry;
    {
        const auto map=MappedLayers();const auto report=geometry.InitializeMapped(map);
        ASSERT_TRUE(report)<<report.message;
        EXPECT_TRUE(geometry.mapping()->Matches(map));
    } // The geometry owns immutable source handles after every caller is gone.
    ASSERT_NO_FATAL_FAILURE(Check(geometry,geometry.mapping()));
    const auto x=geometry.positions();EXPECT_EQ(x.at(0).x,100);EXPECT_EQ(x.at(0).z,300);
    for(unsigned i=0;i<geometry.weights()->node_count();++i) {
        EXPECT_NE(geometry.weights()->node(i).node,0u);
        EXPECT_NE(geometry.weights()->node(i).node,6u);
    }
    for(unsigned i=0;i<x.node_count;++i) {
        const auto expected=geometry.mapping()->domain()->nodes()[i].position;
        EXPECT_EQ(x.at(i).x,expected.x);EXPECT_EQ(x.at(i).y,expected.y);EXPECT_EQ(x.at(i).z,expected.z);
    }
}
TEST(MappedContactGeometry, ExactBoundAndInvalidMapLeaveFreshOrAcceptedGeometryUntouched) {
    const auto map=MappedLayers();fe::ShellNodeMap invalid;ShellCollectionContactGeometry geometry;
    EXPECT_EQ(geometry.InitializeMapped(invalid).status,ShellContactGeometryStatus::InvalidInput);
    auto limits=ShellContactGeometryLimits{};limits.max_startup_bytes=1;
    EXPECT_EQ(geometry.InitializeMapped(map,limits).status,ShellContactGeometryStatus::ResourceLimit);
    EXPECT_FALSE(geometry.prepared());ASSERT_TRUE(geometry.InitializeMapped(map));
    const auto* accepted=geometry.weights();
    EXPECT_EQ(geometry.InitializeMapped(invalid).status,ShellContactGeometryStatus::InvalidInput);
    EXPECT_EQ(geometry.InitializeMapped(map).status,ShellContactGeometryStatus::InvalidInput);
    EXPECT_EQ(geometry.weights(),accepted);
    limits={};limits.max_startup_bytes=geometry.startup_payload_bytes();
    ShellCollectionContactGeometry exact;ASSERT_TRUE(exact.InitializeMapped(map,limits));
    --limits.max_startup_bytes;ShellCollectionContactGeometry short_budget;
    EXPECT_EQ(short_budget.InitializeMapped(map,limits).status,ShellContactGeometryStatus::ResourceLimit);
    EXPECT_FALSE(short_budget.prepared());
}
} // namespace crash::cases::test
