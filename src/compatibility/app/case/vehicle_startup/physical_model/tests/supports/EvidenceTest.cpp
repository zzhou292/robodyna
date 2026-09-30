#include "Support.h"
#include <map>
namespace crash::cases::vehicle_startup::physical_model::supports_test {
TEST(VehicleSupportsPhysicalOriginal, BeamEidEvidenceIsCompleteAndN3CreatesNoEndpointRole) {
    namespace scope=modelio::physical_scope;
    const auto& data=Scope().data(); const auto& beam=Beams().data();
    std::map<std::uint64_t,const scope::NodeEvidence*> evidence;
    for (const auto& node:data.evidence) evidence.emplace(node.node,&node);
    for (const auto& row:beam.rows) for (unsigned slot=0;slot<3;++slot) {
        const auto& node=beam.nodes[row.nodes[slot]];
        const auto found=evidence.find(node.id); ASSERT_NE(found,evidence.end());
        bool hit=false;
        for (const auto& occurrence:found->second->incidence) {
            if (occurrence.family!=scope::Family::Beam || occurrence.element!=row.element_id ||
                !(occurrence.local_slots & (1u<<slot))) continue;
            hit=true; EXPECT_EQ(occurrence.part,row.part_id); EXPECT_EQ(occurrence.canonical_row,row.canonical_row);
            EXPECT_EQ(occurrence.orientation_only,slot==2); EXPECT_EQ(occurrence.selected,slot<2);
        }
        EXPECT_TRUE(hit);
        const bool endpoint=std::binary_search(beam.canonical_endpoints.begin(),beam.canonical_endpoints.end(),node.canonical_index);
        EXPECT_EQ(bool(data.node_roles[node.canonical_index]&scope::Beam18Endpoint),endpoint);
    }
}
} // namespace crash::cases::vehicle_startup::physical_model::supports_test
