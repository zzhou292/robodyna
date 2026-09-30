#include "../Internal.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <set>
#include <sstream>

namespace crash::cases::vehicle_startup::cin_actual_test {
const TiedCinAttachments& Prepared();
namespace {
template<class T> std::string Ids(const std::set<T>& values) {
    std::ostringstream text;
    for (const auto value : values) {
        if (text.tellp() > 0) text << ',';
        text << value;
    }
    return text.str();
}
}
TEST(TiedCinActual, SelectedMechanicalMasterRoleCensus) {
    const auto& value = Prepared();
    const auto& model = value.model();
    const auto& context = value.post_kinchk().classification().context();
    const auto& rigid = context.rigid();
    const auto& declaration = context.auxiliary().declaration().data();
    // The immutable source keeps declaration/member traversal order. Use a
    // test-local lookup copy without changing that source authority or read set.
    auto plain = rigid.data().plain_rigid_members;
    std::sort(plain.begin(),plain.end());
    ASSERT_EQ(model.rows().count,11165u);
    std::set<tied::SourceId> selected_nodes, rigid_nodes, plain_nodes, root_parts;
    std::set<tied::SourceId> plain_groups, auxiliary_groups, auxiliary_members, auxiliary_selected;
    for (const auto& group : context.auxiliary().data().groups) {
        auxiliary_members.insert(group.evidence.source_nodes.begin(),group.evidence.source_nodes.end());
    }
    std::size_t rigid_rows = 0, plain_rows = 0, rigid_slots = 0, plain_slots = 0;
    std::size_t auxiliary_rows = 0, auxiliary_slots = 0;
    std::size_t secondary_rigid = 0, secondary_plain = 0, secondary_auxiliary = 0;
    for (std::size_t row = 0; row < model.rows().count; ++row) {
        const auto& attachment = model.rows().data[row];
        const auto secondary = model.domain()->nodes()[attachment.secondary_domain_node].source_id;
        secondary_rigid += rigid.root_for_node(secondary) != SIZE_MAX;
        secondary_plain += std::binary_search(plain.begin(),plain.end(),secondary);
        secondary_auxiliary += auxiliary_members.count(secondary);
        bool touches_rigid = false, touches_plain = false, touches_auxiliary = false;
        for (const auto index : attachment.master_domain_nodes) {
            const auto id = model.domain()->nodes()[index].source_id;
            selected_nodes.insert(id);
            const auto root = rigid.root_for_node(id);
            if (root != SIZE_MAX) {
                ASSERT_LT(root,rigid.topology().root_count());
                const auto part = rigid.topology().roots()[root].part_index;
                ASSERT_LT(part,rigid.topology().part_count());
                root_parts.insert(rigid.topology().parts()[part].source_part_id);
                rigid_nodes.insert(id);
                ++rigid_slots;
                touches_rigid = true;
            }
            if (std::binary_search(plain.begin(),plain.end(),id)) {
                plain_nodes.insert(id);
                ++plain_slots;
                touches_plain = true;
            }
            if (auxiliary_members.count(id)) {
                auxiliary_selected.insert(id);
                ++auxiliary_slots;
                touches_auxiliary = true;
            }
        }
        rigid_rows += touches_rigid;
        plain_rows += touches_plain;
        auxiliary_rows += touches_auxiliary;
    }
    for (const auto& group : declaration.groups) {
        for (const auto id : group.source_nodes) {
            if (selected_nodes.count(id)) plain_groups.insert(group.id);
        }
    }
    for (const auto& group : context.auxiliary().data().groups) {
        for (const auto id : group.evidence.source_nodes) {
            if (selected_nodes.count(id)) auxiliary_groups.insert(group.evidence.id);
        }
    }
    // This is the source-readset condition already independently qualified by
    // classification. Master roles are observations, not guessed zero inputs.
    EXPECT_EQ(secondary_rigid,0u);
    EXPECT_EQ(secondary_plain,0u);
    EXPECT_EQ(secondary_auxiliary,0u);
    RecordProperty("selected_master_nodes",std::to_string(selected_nodes.size()));
    RecordProperty("selected_rigid_part_master_rows",std::to_string(rigid_rows));
    RecordProperty("selected_rigid_part_master_slot_occurrences",std::to_string(rigid_slots));
    RecordProperty("selected_rigid_part_master_nodes",std::to_string(rigid_nodes.size()));
    RecordProperty("selected_rigid_root_part_ids",Ids(root_parts));
    RecordProperty("selected_plain_group_master_rows",std::to_string(plain_rows));
    RecordProperty("selected_plain_group_master_slot_occurrences",std::to_string(plain_slots));
    RecordProperty("selected_plain_group_master_nodes",std::to_string(plain_nodes.size()));
    RecordProperty("selected_plain_group_ids",Ids(plain_groups));
    RecordProperty("selected_auxiliary_group_ids",Ids(auxiliary_groups));
    RecordProperty("selected_auxiliary_group_master_rows",std::to_string(auxiliary_rows));
    RecordProperty("selected_auxiliary_group_master_slot_occurrences",std::to_string(auxiliary_slots));
    RecordProperty("selected_auxiliary_group_master_nodes",std::to_string(auxiliary_selected.size()));
    RecordProperty("selected_secondary_rigid_members",std::to_string(secondary_rigid));
    RecordProperty("selected_secondary_plain_members",std::to_string(secondary_plain));
    RecordProperty("selected_secondary_auxiliary_members",std::to_string(secondary_auxiliary));
}
}
