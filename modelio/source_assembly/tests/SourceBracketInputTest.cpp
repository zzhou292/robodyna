#include "AssemblyTestSupport.h"
#include <algorithm>
#include <cmath>
#include <set>

namespace crash::modelio::assembly::test {
namespace {
std::string BracketBytes() {
    const auto* path = std::getenv("ROBO_DYNA_SOURCE_BRACKET_INVENTORY");
    output::Require(path && *path, "Explicit original bracket fixture is required");
    return output::ReadBounded(path, PinnedYarisSevenPartInventory().bytes);
}
SourceAssembly Bracket() { return SourceAssembly::ReadBytes(BracketBytes(), PinnedYarisSevenPartInventory()); }
void RejectBracket(const std::function<void(output::Document&)>& edit) {
    const auto bytes = BracketBytes(); output::Document document;
    document.Parse<rapidjson::kParseFullPrecisionFlag>(bytes.data(), bytes.size());
    ASSERT_FALSE(document.HasParseError()); edit(document);
    rapidjson::StringBuffer buffer; rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    ASSERT_TRUE(document.Accept(writer)); const std::string altered(buffer.GetString(), buffer.GetSize());
    EXPECT_THROW(SourceAssembly::ReadBytes(altered, ExplicitTestIdentity(altered)), std::runtime_error);
}
output::Value& InternalRecord(output::Document& document) {
    for (auto& row : document["attachments"]["spotwelds"].GetArray())
        if (std::string(row["classification"].GetString()) == "internal") return row;
    throw std::runtime_error("Actual source bracket has no internal spotweld");
}
}

TEST(SourceBracketInput, CompleteOriginalBracketAndFiniteOffsetWeldAreRetained) {
    const auto input = Bracket(); const auto& d = input.data();
    EXPECT_EQ(d.nodes.size(), 1093u); EXPECT_EQ(d.parents.size(), 959u);
    EXPECT_EQ(d.qeph_count, 845u); EXPECT_EQ(d.t3_count, 114u);
    EXPECT_EQ(d.parts.size(), 7u); EXPECT_EQ(d.materials.size(), 7u); EXPECT_EQ(d.sections.size(), 7u);
    EXPECT_EQ(d.curves.size(), 2u);
    const auto bracket = std::find_if(d.parts.begin(), d.parts.end(), [](const auto& p) { return p.id == 2000204; });
    ASSERT_NE(bracket, d.parts.end()); EXPECT_EQ(bracket->parent_count, 44u); EXPECT_EQ(bracket->nodes.size(), 63u);
    ASSERT_EQ(d.internal_spotwelds.size(), 1u); const auto& weld = d.internal_spotwelds.front();
    EXPECT_EQ(weld.record.id, 2101297u);
    EXPECT_EQ(weld.record.node_ids, (std::array<SourceId,2>{2181504,2204838}));
    EXPECT_EQ(weld.record.cards[0].source_line, 26562u); EXPECT_EQ(weld.record.cards[1].source_line, 26563u);
    EXPECT_EQ(weld.record.cards[1].blank_mask, 252u); EXPECT_TRUE(weld.record.external_nodes.empty());
    ASSERT_EQ(weld.record.selected_membership.size(), 2u);
    EXPECT_EQ(weld.record.selected_membership[0].part_id, 2000145u);
    EXPECT_EQ(weld.record.selected_membership[1].part_id, 2000204u);
    for (unsigned i=0;i<2;++i) EXPECT_EQ(d.nodes[weld.nodes[i]].source_id, weld.record.node_ids[i]);
    const auto& first=d.nodes[weld.nodes[0]].position_m;
    const auto& second=d.nodes[weld.nodes[1]].position_m;
    const double distance=std::hypot(first.x-second.x,first.y-second.y,first.z-second.z);
    EXPECT_GT(distance, .003); EXPECT_LT(distance, .011);
    EXPECT_EQ(d.released_spotwelds.size(), 13u); EXPECT_EQ(d.boundary.nodal_rigid_ids.size(), 8u);
    EXPECT_EQ(d.boundary.external_node_ids.size(), 53u); EXPECT_EQ(d.boundary.external_part_ids.size(), 10u);
    EXPECT_EQ(std::count_if(d.nodal_rigid_groups.begin(), d.nodal_rigid_groups.end(), [](const auto& g){return g.internal;}), 6);
    EXPECT_EQ(std::count(d.boundary.spotweld_ids.begin(),d.boundary.spotweld_ids.end(),2101297u),0);
}

TEST(SourceBracketInput, EveryPriorPhysicalNodeAndShellRemainsInTheCompleteExtension) {
    const auto old=Load(), next=Bracket(); const auto& a=old.data(); const auto& b=next.data();
    for (const auto& node:a.nodes) {
        const auto found=std::lower_bound(b.nodes.begin(),b.nodes.end(),node.source_id,
            [](const auto& n, SourceId id){return n.source_id<id;});
        ASSERT_NE(found,b.nodes.end()); ASSERT_EQ(found->source_id,node.source_id);
        SameBits(found->position_m.x,node.position_m.x); SameBits(found->position_m.y,node.position_m.y);
        SameBits(found->position_m.z,node.position_m.z);
        EXPECT_EQ(found->codes,node.codes); EXPECT_EQ(found->canonical_index,node.canonical_index);
    }
    for (const auto& parent:a.parents) {
        const auto found=std::find_if(b.parents.begin(),b.parents.end(),[&](const auto& p){return p.source_id==parent.source_id;});
        ASSERT_NE(found,b.parents.end()); EXPECT_EQ(found->raw_record,parent.raw_record);
        EXPECT_EQ(found->part_id,parent.part_id); EXPECT_EQ(found->material_id,parent.material_id);
        EXPECT_EQ(found->section_id,parent.section_id); EXPECT_EQ(found->curve_id,parent.curve_id);
        for (unsigned n=0;n<parent.arity;++n)
            EXPECT_EQ(b.nodes[found->nodes[n]].source_id,a.nodes[parent.nodes[n]].source_id);
    }
    const SourceAssemblyShellInput shells(next);
    const SourceAssemblyMaterialInput materials(next,MaterialRatePolicy::OpenRadiossDirectImportDefault);
    EXPECT_EQ(shells.qeph_source_parents().size(),845u); EXPECT_EQ(shells.t3_source_parents().size(),114u);
}

TEST(SourceBracketInput, InternalMembershipBoundaryAndLiteralIdentityCannotBeChanged) {
    RejectBracket([](auto& d){InternalRecord(d)["classification"].SetString("outgoing",d.GetAllocator());});
    RejectBracket([](auto& d){d["counts"]["internal_spotwelds"].SetUint(0);});
    RejectBracket([](auto& d){InternalRecord(d)["weld"]["node_ids"][1].SetUint64(2181504);});
    RejectBracket([](auto& d){InternalRecord(d)["selected_membership"][1]["node_ids"][0].SetUint64(2181504);});
    RejectBracket([](auto& d){d["boundary"]["outgoing_spotweld_ids"].PopBack();});
    RejectBracket([](auto& d){auto& rows=d["attachments"]["spotwelds"]; output::Value copy;
        copy.CopyFrom(InternalRecord(d),d.GetAllocator()); rows.PushBack(copy,d.GetAllocator());});
    EXPECT_EQ(Bracket().data().internal_spotwelds.front().record.id,2101297u);
}

TEST(SourceBracketInput, FrontierCapacityIsIndependentFromActivePartCapacity) {
    const auto bytes=BracketBytes();
    for (unsigned failure=0;failure<4;++failure) {
        ReadLimits limits;
        if (failure==0) limits.external_parts=9;
        if (failure==1) limits.external_parts=33;
        if (failure==2) limits.external_parts=0;
        if (failure==3) limits.parts=6;
        EXPECT_THROW(SourceAssembly::ReadBytes(bytes,PinnedYarisSevenPartInventory(),limits),std::runtime_error);
    }
    EXPECT_EQ(Bracket().data().parts.size(),7u);
}
} // namespace crash::modelio::assembly::test
