#include "OriginalSupport.h"
#include <map>

namespace crash::cases::vehicle_startup::shell_execution::test {
TEST(VehicleShellExecutionOriginal, AllThreeOriginalWindshieldLayersKeepDistinctParentsPlacementAndPointRoles) {
    const auto& actual = Actual();
    const auto& binding = *actual.physical().shells();
    using Key = std::array<std::size_t,5>;
    std::map<Key,std::array<const fe::ShellExecutionParent*,3>> layers;
    for (std::size_t e = 0; e < actual.catalog().parent_count(); ++e) {
        const auto* parent = actual.execution().parent(e);
        const auto& source = parent->source;
        const auto pid = source.source_part_id;
        if (pid != 2000023 && pid != 2000523 && pid != 2000524) continue;
        Key key{0,SIZE_MAX,SIZE_MAX,SIZE_MAX,SIZE_MAX};
        if (source.family == fe::ShellBindingFamily::T3) {
            key[0] = 3;
            const auto& nodes = binding.t3_nodes(source.family_index);
            std::copy(nodes.begin(),nodes.end(),key.begin()+1);
        } else {
            key[0] = 4;
            const auto& nodes = source.family == fe::ShellBindingFamily::Qbat ?
                binding.qbat_nodes(source.family_index) : binding.qeph_nodes(source.family_index);
            std::copy(nodes.begin(),nodes.end(),key.begin()+1);
        }
        std::sort(key.begin()+1,key.end());
        const unsigned slot = pid == 2000023 ? 0 : (pid == 2000523 ? 1 : 2);
        ASSERT_EQ(layers[key][slot],nullptr);
        layers[key][slot] = parent;
    }
    ASSERT_EQ(layers.size(),4251);
    unsigned triangles = 0;
    for (const auto& [key, rows] : layers) {
        for (const auto* row : rows) ASSERT_NE(row,nullptr);
        EXPECT_NE(rows[0]->source.source_parent_id,rows[1]->source.source_parent_id);
        EXPECT_NE(rows[0]->source.source_parent_id,rows[2]->source.source_parent_id);
        EXPECT_NE(rows[1]->source.source_parent_id,rows[2]->source.source_parent_id);
        EXPECT_EQ(rows[0]->material_points,3);
        EXPECT_EQ(rows[1]->material_points,3);
        const auto placement = [&](unsigned i) {
            const auto& source = rows[i]->source;
            return key[0] == 3 ? binding.t3_reference(source.family_index).input.placement :
                binding.qeph_reference(source.family_index).input.placement;
        };
        EXPECT_NE(placement(0),placement(1));
        EXPECT_NE(placement(0),fe::ShellReferencePlacement::Centered);
        EXPECT_NE(placement(1),fe::ShellReferencePlacement::Centered);
        EXPECT_EQ(rows[2]->source.material_id,2000524);
        EXPECT_EQ(rows[2]->source.section_id,2000524);
        if (key[0] == 3) {
            ++triangles;
            EXPECT_EQ(rows[2]->source.source_parent_id,2357656);
            EXPECT_EQ(rows[2]->law,fe::ShellSectionLaw::Law44Nip1);
            EXPECT_EQ(rows[2]->material_points,1);
        } else {
            EXPECT_EQ(rows[2]->law,fe::ShellSectionLaw::Law44QbatFourInPlane);
            EXPECT_EQ(rows[2]->material_points,4);
        }
    }
    EXPECT_EQ(triangles,1);
}
} // namespace crash::cases::vehicle_startup::shell_execution::test
