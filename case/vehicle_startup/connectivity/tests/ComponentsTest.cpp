#include "Fixture.h"

namespace crash::cases::vehicle_startup::connectivity::test {
TEST(VehicleConnectivityValues, TwoPartitionsMatchIndependentBfsAndReverseOrder) {
    Fixture source;
    auto data=source.Build();
    detail::Partition(source.Nodes(),data);
    EXPECT_EQ(data.element_label,Bfs(source,false));
    EXPECT_EQ(data.transfer_label,Bfs(source,true));
    EXPECT_EQ(data.counts.element_components,7u);
    EXPECT_EQ(data.counts.transfer_components,2u);
    auto reverse=source.Build(true);
    detail::Partition(source.Nodes(),reverse);
    EXPECT_EQ(reverse.element_label,data.element_label);
    EXPECT_EQ(reverse.transfer_label,data.transfer_label);
    EXPECT_EQ(reverse.node_roles,data.node_roles);
    EXPECT_EQ(data.counts.by_kind[static_cast<std::size_t>(Kind::PointMass)],2u);
}
TEST(VehicleConnectivityValues, N3CoordinatesMassAndRigidSkinDoNotInventElementEdges) {
    Fixture source;
    auto data=source.Build();
    detail::Partition(source.Nodes(),data);
    EXPECT_NE(data.element_label[2],data.element_label[8]); // N3 excluded.
    EXPECT_NE(data.element_label[3],data.element_label[8]); // Equal coordinates ignored.
    EXPECT_NE(data.element_label[7],data.element_label[9]); // Skin is nonconstitutive.
    EXPECT_EQ(data.element_label[9],2u); // Literal mass adds no edge.
    EXPECT_EQ(data.transfer_label[18],1u); // Mass-only node retained.
    EXPECT_EQ(data.counts.mass_without_element_incidence,2u);
    EXPECT_EQ(data.counts.rigid_skin_parents,1u);
    const auto& cin=data.relations.back();
    ASSERT_EQ(cin.slot_count,5u);
    EXPECT_EQ(data.slots[cin.slot_offset+3],data.slots[cin.slot_offset+4]);
    EXPECT_EQ(data.transfer_label[5],data.transfer_label[10]);
}
TEST(VehicleConnectivityValues, LateInvalidIndexAndTypedWidthPreservePreviousLabelsThenRetry) {
    Fixture source;
    auto data=source.Build();
    const auto before=data.element_label, transfer=data.transfer_label;
    const auto roles=data.node_roles;
    data.slots.back()=source.nodes.size();
    EXPECT_THROW(detail::Partition(source.Nodes(),data),std::runtime_error);
    EXPECT_EQ(data.element_label,before); EXPECT_EQ(data.transfer_label,transfer); EXPECT_EQ(data.node_roles,roles);
    data.slots.back()=9;
    data.relations.back().kind=Kind::T3;
    EXPECT_THROW(detail::Partition(source.Nodes(),data),std::runtime_error);
    EXPECT_EQ(data.element_label,before);
    data.relations.back().kind=Kind::Cin;
    ASSERT_NO_THROW(detail::Partition(source.Nodes(),data));
    EXPECT_EQ(data.transfer_label,Bfs(source,true));
}
TEST(VehicleConnectivityValues, CompleteCompactReportHasExactCapAndPreservesSourceRoles) {
    Fixture source;
    auto data=source.Build();
    detail::Partition(source.Nodes(),data);
    const detail::ReportIdentity identity{std::string(64,'a'),std::string(64,'b'),std::string(64,'c'),"tiny"};
    Forecast forecast;
    const auto text=detail::RenderReport(data,source.Nodes(),forecast,identity,Limits{}.report_bytes);
    ASSERT_EQ(detail::RenderReport(data,source.Nodes(),forecast,identity,text.size()),text);
    EXPECT_THROW(detail::RenderReport(data,source.Nodes(),forecast,identity,text.size()-1),std::runtime_error);
    output::Document doc;
    doc.Parse(text.data(),text.size());
    ASSERT_FALSE(doc.HasParseError());
    ASSERT_EQ(doc["nodes"].Size(),source.nodes.size());
    ASSERT_EQ(doc["relations"].Size(),source.relations.size());
    EXPECT_EQ(doc["counts"]["joint_edges"].GetUint64(),0u);
    EXPECT_EQ(doc["relations"][2][1].GetUint(),static_cast<unsigned>(Role::RigidSkin));
    EXPECT_EQ(doc["relations"][2][6].GetUint(),0u);
    EXPECT_EQ(doc["relations"][10][5].Size(),5u);
}
} // namespace crash::cases::vehicle_startup::connectivity::test
