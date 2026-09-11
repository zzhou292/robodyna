#include "OriginalSupport.h"
#include "case/shell_collection/ShellCollectionContactGeometry.h"
#include <algorithm>

namespace crash::cases::vehicle_startup::shell_execution::test {
TEST(VehicleShellExecutionOriginal, CompleteMappedContactSurfaceKeepsEveryFamilyAndOriginalParent) {
    const auto& actual=Actual();const auto& map=*actual.physical().mapping();
    ShellCollectionContactGeometry geometry;
    const auto report=geometry.InitializeMapped(map,ShellContactGeometryLimits::Vehicle());
    ASSERT_TRUE(report)<<report.message<<" source EID "<<report.parent.source_id;
    ASSERT_TRUE(geometry.mapping()->Matches(map));
    const auto& weights=*geometry.weights();
    ASSERT_EQ(weights.parent_count(),349645u);ASSERT_EQ(weights.node_count(),359785u);
    ASSERT_EQ(weights.global_node_count(),372435u);
    std::size_t q=0,t=0,b=0,skin=0;
    for(std::size_t p=0;p<weights.parent_count();++p) {
        const auto* key=geometry.parent_from_weight(p);ASSERT_NE(key,nullptr);
        const auto* entry=actual.execution().parent(key->family,key->family_index);ASSERT_NE(entry,nullptr);
        EXPECT_EQ(entry->source.source_parent_id,key->source_id);
        EXPECT_EQ(weights.parent(p).parent_element_id,key->source_id);
        const bool tri=key->family==fe::ShellBindingFamily::T3;
        const bool qb=key->family==fe::ShellBindingFamily::Qbat;
        if(tri)++t;else if(qb)++b;else ++q;
        if(entry->law==fe::ShellSectionLaw::RigidSkin)++skin;
        EXPECT_EQ(weights.parent(p).arity,tri?3u:4u);
        for(unsigned l=0;l<weights.parent(p).arity;++l) {
            const auto local=tri?map.shells()->t3_nodes(key->family_index)[l]:
                qb?map.shells()->qbat_nodes(key->family_index)[l]:map.shells()->qeph_nodes(key->family_index)[l];
            EXPECT_EQ(weights.parent(p).nodes[l],map.owner_index(local));
        }
    }
    EXPECT_EQ(q,324094u);EXPECT_EQ(t,21301u);EXPECT_EQ(b,4250u);EXPECT_EQ(skin,5102u);
    std::vector<std::size_t> expected(map.mapping().data,map.mapping().data+map.mapping().count);
    std::sort(expected.begin(),expected.end());
    for(std::size_t n=0;n<expected.size();++n)EXPECT_EQ(weights.node(n).node,expected[n]);
    RecordProperty("contact_parents",std::to_string(weights.parent_count()));
    RecordProperty("surface_nodes",std::to_string(weights.node_count()));
    RecordProperty("physical_nodes",std::to_string(weights.global_node_count()));
    RecordProperty("contact_startup_payload_bytes",std::to_string(geometry.startup_payload_bytes()));
    RecordProperty("contact_weight_payload_bytes",std::to_string(weights.owned_payload_bytes()));
    RecordProperty("scope","complete immutable mapped original surface; no runtime contact admission");
}
} // namespace crash::cases::vehicle_startup::shell_execution::test
