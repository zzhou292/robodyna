#include "V5Fixture.h"

namespace crash::cases::vehicle_startup::connectivity::test {
TEST(VehicleConnectivityV5, CompleteTypedCountsAndEveryExtendedSourceSlot) {
    const auto& value = ActualV5();
    const auto& data = value.data();
    const auto& physical = value.source().physical();
    const auto& domain = physical.source_domain().domain();
    const auto& solids = physical.solids();
    const auto count = [&](Kind kind) { return data.counts.by_kind[static_cast<std::size_t>(kind)]; };
    EXPECT_EQ(data.counts.nodes,376930u); EXPECT_EQ(data.counts.relations,374179u);
    EXPECT_EQ(count(Kind::Qeph)+count(Kind::T3)+count(Kind::Qbat),349645u);
    EXPECT_EQ(count(Kind::Solid18),908u); EXPECT_EQ(count(Kind::Solid24),1991u);
    EXPECT_EQ(count(Kind::Solid6z),350u); EXPECT_EQ(count(Kind::Solid18Law44),386u);
    EXPECT_EQ(count(Kind::Solid18Law90),1345u); EXPECT_EQ(count(Kind::Beam18),142u);
    EXPECT_EQ(count(Kind::Type13),4442u); EXPECT_EQ(count(Kind::Type25),2828u);
    EXPECT_EQ(count(Kind::PartRoot),20u); EXPECT_EQ(count(Kind::PlainGroup),759u);
    EXPECT_EQ(count(Kind::PointMass),154u); EXPECT_EQ(count(Kind::Cin),11165u);
    EXPECT_EQ(count(Kind::SphericalJoint)+count(Kind::RevoluteJoint)+count(Kind::CylindricalJoint),44u);
    ASSERT_NE(value.joint_model(),nullptr);
    const auto& joints = *value.joint_model();
    EXPECT_EQ(joints.source().data().boundaries,0u);
    EXPECT_TRUE(joints.physical().SharesStorage(physical));
    EXPECT_EQ(value.required_joints(),Obligation::PreparedSourceOperators);
    std::size_t extended = 0, beams = 0, joint_rows = 0, repeated = 0;
    const auto check = [&](const Relation& row, const auto& parent) {
        const auto& input = parent.reference.input();
        EXPECT_EQ(row.source_id,input.source_element_id); EXPECT_EQ(row.part_id,input.source_part_id);
        for (std::size_t slot = 0; slot < row.slot_count; ++slot) {
            const auto node = data.slots[row.slot_offset+slot];
            EXPECT_EQ(node,parent.domain_nodes[slot]);
            EXPECT_EQ(domain.nodes()[node].source_id,input.source_node_id[slot]);
        }
    };
    for (const auto& row : data.relations) {
        if (row.kind == Kind::Solid18Law44 || row.kind == Kind::Solid18Law90) {
            const auto& source = physical.source_domain().source().solid_source().data().rows.at(row.source_row);
            if (row.kind == Kind::Solid18Law44) {
                check(row,solids.solid18_law44()[source.reference_index]);
                repeated += data.slots[row.slot_offset+4] == data.slots[row.slot_offset+5];
            } else check(row,solids.solid18_law90()[source.reference_index]);
            ++extended;
        } else if (row.kind == Kind::Beam18) {
            ASSERT_EQ(row.slot_count,2u);
            check(row,physical.structural_beams()->parents()[row.source_row]);
            ++beams;
        } else if (IsJoint(row.kind)) {
            ASSERT_LT(joint_rows,joints.model().joints().size());
            const auto& joint = joints.model().joints()[joint_rows];
            const auto& original = joints.source().data().rows.at(row.source_row);
            ASSERT_EQ(row.slot_count,2u);
            EXPECT_EQ(row.source_row,joints.source_rows()[joint_rows]);
            EXPECT_EQ(row.source_id,original.source_id);
            EXPECT_EQ(row.source_id,joint.geometry.source_joint_id);
            const unsigned expected = joint.property.kind == tl::fea::type45::Kind::Spherical ? 56 :
                joint.property.kind == tl::fea::type45::Kind::Revolute ? 8 : 9;
            EXPECT_EQ(ReleasedDofs(row.kind),expected);
            for (unsigned end = 0; end < 2; ++end) {
                EXPECT_EQ(data.slots[row.slot_offset+end],joint.domain_nodes[end]);
                EXPECT_EQ(data.slots[row.slot_offset+end],original.nodes[end].domain_index);
            }
            ++joint_rows;
        } else if (row.kind == Kind::Cin) {
            const auto& source = value.source().attachments().model().rows().data[row.source_row];
            EXPECT_EQ(data.slots[row.slot_offset],source.secondary_domain_node);
            for (unsigned slot = 0; slot < 4; ++slot)
                EXPECT_EQ(data.slots[row.slot_offset+1+slot],source.master_domain_nodes[slot]);
        }
    }
    EXPECT_EQ(extended,1731u); EXPECT_EQ(beams,142u); EXPECT_EQ(joint_rows,44u);
    EXPECT_EQ(repeated,113u); //109 rear collapsed cells +4 explicit airbag cells.
}
TEST(VehicleConnectivityV5, ExactCapsAndImmutableJointLifetimeSurviveFailedReplacement) {
    const auto& value = ActualV5();
    const auto& joints = *value.joint_model();
    auto limits = Limits{};
    limits.total_bytes = value.forecast().total_bytes-1;
    const auto before = value.data().transfer_label.back();
    EXPECT_THROW(VehicleConnectivity::PrepareWithJoints(value.source(),joints,limits),std::runtime_error);
    EXPECT_EQ(value.data().transfer_label.back(),before);
    ++limits.total_bytes;
    EXPECT_EQ(VehicleConnectivity::PreflightWithJoints(value.source(),joints,limits).total_bytes,
              value.forecast().total_bytes);
    limits.extra_bytes = value.forecast().extra_bytes-1;
    EXPECT_THROW(VehicleConnectivity::PreflightWithJoints(value.source(),joints,limits),std::runtime_error);
    ++limits.extra_bytes;
    EXPECT_EQ(VehicleConnectivity::PreflightWithJoints(value.source(),joints,limits).extra_bytes,
              value.forecast().extra_bytes);
    auto copy = value;
    EXPECT_EQ(&copy.data(),&value.data());
    EXPECT_TRUE(copy.joint_model()->model().SharesStorage(joints.model()));
    EXPECT_TRUE(copy.joint_model()->physical().SharesStorage(value.source().physical()));
    const auto without = VehicleConnectivity::Preflight(value.source());
    EXPECT_EQ(without.extents.relations+44,value.forecast().extents.relations);
    EXPECT_EQ(without.retained_source_bound+joints.additional_owned_payload_bytes(),
              value.forecast().retained_source_bound);
}
} // namespace crash::cases::vehicle_startup::connectivity::test
