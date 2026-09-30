#include "Fixture.h"
#include "../ElementRows.h"
#include "../JointRows.h"
#include "lib_src/elements/solid18/Solid18Types.h"
#include "lib_src/elements/beam18/Types.h"

namespace crash::cases::vehicle_startup::connectivity::test {
namespace {
struct TypedFixture {
    std::vector<tl::fea::NodalDomainNode> nodes;
    Data data;
    Counts expected;
    TypedFixture(std::size_t count, std::size_t rows, std::size_t slots) {
        for (std::size_t node = 0; node < count; ++node)
            nodes.push_back({1000+17*(count-node),{double(node),0,0}});
        expected.nodes = count; expected.relations = rows; expected.slots = slots;
        data.element_label.resize(count,999); data.transfer_label.resize(count,999);
        data.node_roles.resize(count,0xff);
    }
    auto Nodes() const { return tl::util::ConstView<tl::fea::NodalDomainNode>{nodes.data(),nodes.size()}; }
    template<class Input, std::size_t N>
    Input InputAt(std::uint64_t id, const std::size_t (&slots)[N]) const {
        Input input;
        input.source_element_id = id; input.source_part_id = 77;
        for (std::size_t slot = 0; slot < N; ++slot) input.source_node_id[slot] = nodes[slots[slot]].source_id;
        return input;
    }
    tl::fea::type45::Joint Joint(tl::fea::type45::Kind kind, std::size_t first, std::size_t last) const {
        tl::fea::type45::Joint joint;
        joint.property.kind = kind;
        joint.geometry.source_joint_id = 700+first;
        joint.domain_nodes[0] = first; joint.domain_nodes[1] = last; joint.domain_nodes[2] = nodes.size()-1;
        for (unsigned slot = 0; slot < 3; ++slot)
            joint.geometry.source_node_id[slot] = nodes[joint.domain_nodes[slot]].source_id;
        return joint;
    }
};
}
TEST(VehicleConnectivityTypedRows, ExtendedSolidSlotsAndBeamEndpointsJoinOnlyTheirActualSupports) {
    TypedFixture f(15,3,18);
    detail::Relations rows(f.data,f.expected);
    const std::size_t rear[]{0,1,2,3,4,4,5,5}, foam[]{6,7,8,9,10,11,12,13}, beam[]{5,6,14};
    const auto metal = f.InputAt<tl::fea::solid18::ReferenceInput>(1,rear);
    const auto rubber = f.InputAt<tl::fea::solid18::ReferenceInput>(2,foam);
    const auto rod = f.InputAt<tl::fea::beam18::Input>(3,beam);
    detail::AppendElementInput(rows,Kind::Solid18Law44,3,metal,f.Nodes(),rear);
    detail::AppendElementInput(rows,Kind::Solid18Law90,4,rubber,f.Nodes(),foam);
    detail::AppendElementInput(rows,Kind::Beam18,5,rod,f.Nodes(),beam);
    rows.Complete();
    ASSERT_EQ(f.data.slots.size(),18u);
    for (unsigned slot = 0; slot < 8; ++slot) EXPECT_EQ(f.data.slots[slot],rear[slot]);
    EXPECT_EQ(f.data.slots[16],5u); EXPECT_EQ(f.data.slots[17],6u);
    detail::Partition(f.Nodes(),f.data);
    for (unsigned node = 0; node < 14; ++node)
        EXPECT_EQ(f.data.element_label[node],f.nodes[13].source_id);
    EXPECT_NE(f.data.element_label[13],f.data.element_label[14]); // N3 has no mechanical edge.
    EXPECT_EQ(f.data.counts.element_components,2u);
    // Remove the actual bridge, retaining the same two solids. This must split
    // the result; no PID-wide or hardcoded expected-component relation exists.
    f.data.relations.pop_back(); f.data.slots.resize(16);
    detail::Partition(f.Nodes(),f.data);
    EXPECT_NE(f.data.transfer_label[5],f.data.transfer_label[6]);
    EXPECT_EQ(f.data.counts.transfer_components,3u);
}
TEST(VehicleConnectivityTypedRows, LateSourceSlotMismatchDoesNotAppendThenRetryPreservesRepeats) {
    TypedFixture f(6,1,8);
    detail::Relations rows(f.data,f.expected);
    std::size_t slots[]{0,1,2,3,4,4,5,5};
    const auto input = f.InputAt<tl::fea::solid18::ReferenceInput>(1,slots);
    slots[7] = 4;
    EXPECT_THROW(detail::AppendElementInput(rows,Kind::Solid18Law44,0,input,f.Nodes(),slots),std::runtime_error);
    EXPECT_TRUE(f.data.relations.empty()); EXPECT_TRUE(f.data.slots.empty());
    EXPECT_EQ(f.data.element_label.front(),999u);
    slots[7] = 5;
    ASSERT_NO_THROW(detail::AppendElementInput(rows,Kind::Solid18Law44,0,input,f.Nodes(),slots));
    rows.Complete();
    detail::Partition(f.Nodes(),f.data);
    EXPECT_EQ(f.data.counts.element_components,1u);
}
TEST(VehicleConnectivityTypedRows, ThreeJointKindsPreserveNativeReleasesAndExcludeAxisNode) {
    TypedFixture f(5,3,6);
    detail::Relations rows(f.data,f.expected);
    using Native = tl::fea::type45::Kind;
    const Native kinds[]{Native::Spherical,Native::Revolute,Native::Cylindrical};
    const unsigned masks[]{56,8,9};
    for (unsigned i = 0; i < 3; ++i) {
        detail::AppendJoint(rows,f.Joint(kinds[i],i,i+1),10+i,f.Nodes());
        EXPECT_EQ(ReleasedDofs(f.data.relations.back().kind),masks[i]);
    }
    rows.Complete();
    detail::Partition(f.Nodes(),f.data);
    EXPECT_EQ(f.data.counts.element_components,5u);
    EXPECT_EQ(f.data.counts.transfer_components,2u);
    for (unsigned node = 0; node < 4; ++node) EXPECT_EQ(f.data.transfer_label[node],f.nodes[3].source_id);
    EXPECT_NE(f.data.transfer_label[4],f.data.transfer_label[0]);
    detail::ReportIdentity identity{"archive","member","canonical","tiny",true,123,0};
    const auto text = detail::RenderReport(f.data,f.Nodes(),{},identity,Limits{}.report_bytes);
    output::Document document; document.Parse(text.data(),text.size());
    ASSERT_FALSE(document.HasParseError());
    EXPECT_STREQ(document["schema"].GetString(),"robo_dyna.original_physical_connectivity.v2");
    EXPECT_EQ(document["counts"]["joint_edges"].GetUint64(),3u);
    EXPECT_TRUE(document["prepared_joint_model"].GetBool());
    for (unsigned i = 0; i < 3; ++i) EXPECT_EQ(document["relations"][i][6].GetUint(),masks[i]);
    EXPECT_EQ(detail::RenderReport(f.data,f.Nodes(),{},identity,text.size()),text);
    EXPECT_THROW(detail::RenderReport(f.data,f.Nodes(),{},identity,text.size()-1),std::runtime_error);
    identity.has_joints = false;
    EXPECT_THROW(detail::RenderReport(f.data,f.Nodes(),{},identity,Limits{}.report_bytes),std::runtime_error);
}
TEST(VehicleConnectivityTypedRows, JointLateEndpointOrFreeStiffnessFailsBeforeAppendingAndRetries) {
    TypedFixture f(5,2,4);
    detail::Relations rows(f.data,f.expected);
    auto first = f.Joint(tl::fea::type45::Kind::Spherical,0,1);
    detail::AppendJoint(rows,first,10,f.Nodes());
    auto last = f.Joint(tl::fea::type45::Kind::Cylindrical,1,2);
    last.domain_nodes[1] = f.nodes.size();
    EXPECT_THROW(detail::AppendJoint(rows,last,11,f.Nodes()),std::runtime_error);
    EXPECT_EQ(f.data.relations.size(),1u); EXPECT_EQ(f.data.slots.size(),2u);
    last.domain_nodes[1] = 2; last.property.free_stiffness.translation.x = 1;
    EXPECT_THROW(detail::AppendJoint(rows,last,11,f.Nodes()),std::runtime_error);
    EXPECT_EQ(f.data.relations.size(),1u); EXPECT_EQ(f.data.slots.size(),2u);
    last.property.free_stiffness.translation.x = 0;
    ASSERT_NO_THROW(detail::AppendJoint(rows,last,11,f.Nodes()));
    rows.Complete();
    detail::Partition(f.Nodes(),f.data);
    EXPECT_EQ(f.data.transfer_label[0],f.data.transfer_label[2]);
    const auto labels = f.data.transfer_label;
    f.data.relations.back().role = Role::Constitutive;
    EXPECT_THROW(detail::Partition(f.Nodes(),f.data),std::runtime_error);
    EXPECT_EQ(f.data.transfer_label,labels); // Released joint is never an element incidence relation.
}
} // namespace crash::cases::vehicle_startup::connectivity::test
