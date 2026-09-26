#include "Fixture.h"
#include <iomanip>
#include <sstream>
namespace crash::cases::vehicle_self_contact::native::initial_surfaces::test {
namespace {
std::string Card(std::initializer_list<std::uint64_t> values) {
    std::ostringstream out;
    for (const auto value : values) out << std::setw(10) << value;
    return out.str();
}
modelio::self_contact::Data SelectionValues() {
    modelio::self_contact::Data out;
    out.source_fields.slave_set_type = 2;
    out.source_fields.slave_set_id = 50;
    out.selected_part_ids = {10,20};
    out.sources.resize(3);
    out.sources[0].block.keyword = "*CONTACT_AUTOMATIC_SINGLE_SURFACE";
    out.sources[1].block.keyword = "*SET_PART_ADD";
    out.sources[1].cards = {{2,Card({50})}, {3,Card({60})}};
    out.sources[2].block.keyword = "*SET_PART_LIST";
    out.sources[2].cards = {{5,Card({60})}, {6,Card({10,20})}};
    return out;
}
}
TEST(InitialSurfaceControls, SinglePositiveChildCopiesOneOrdinaryPartList) {
    auto input = SelectionValues();
    const auto proof = detail::ResolveSelectionControls(input, 16);
    EXPECT_EQ(proof.rule, "a62_direct_fresh_single_list_PART_EXT1_OPT_O0_NADMESH0_NUMELTRIA0_v1");
    input.sources[2].block.keyword = "*SET_PART_LIST_TITLE";
    input.sources[2].cards.insert(input.sources[2].cards.begin(), {4,"contact parts"});
    EXPECT_EQ(detail::ResolveSelectionControls(input, 16).rule, proof.rule);
}
TEST(InitialSurfaceControls, MultipleClausesAndUnclosedOptionsNeverAcquireACertificate) {
    auto input = SelectionValues();
    input.sources[1].cards[1].second = Card({60,70});
    EXPECT_THROW(detail::ResolveSelectionControls(input, 16), detail::Failure);
    input = SelectionValues();
    input.sources[2].block.keyword = "*SET_PART_ADD";
    EXPECT_THROW(detail::ResolveSelectionControls(input, 16), detail::Failure);
    input = SelectionValues();
    input.sources[1].cards[0].second = Card({50,1});
    EXPECT_THROW(detail::ResolveSelectionControls(input, 16), detail::Failure);
    input = SelectionValues();
    input.selected_part_ids = {20,10};
    EXPECT_THROW(detail::ResolveSelectionControls(input, 16), detail::Failure);
    input = SelectionValues();
    input.sources.push_back(input.sources[0]);
    EXPECT_THROW(detail::ResolveSelectionControls(input, 16), detail::Failure);
    input = SelectionValues();
    input.source_fields.master_set_id = 99;
    EXPECT_THROW(detail::ResolveSelectionControls(input, 16), detail::Failure);
    EXPECT_THROW(detail::ResolveSelectionControls(SelectionValues(), 1), detail::Failure);
}
TEST(InitialSurfaceDocuments, FailureHasNoReadyCensusAndBoundedDiagnostics) {
    Preparation failure;
    failure.report = {Status::NeedsNativeReaderOrder, "conflicting early membership", 101, 201, 202, 1};
    const auto document = ResultDocument(failure);
    EXPECT_STREQ(document["status"].GetString(), "needs_native_reader_order");
    EXPECT_EQ(document["diagnostic_solid_eid"].GetUint64(), 101u);
    EXPECT_FALSE(document.HasMember("faces"));
    EXPECT_THROW(ResultDocument(failure, 1), std::exception);
    failure.report.status = Status::Ready;
    EXPECT_THROW(ResultDocument(failure), std::exception);
}
TEST(InitialSurfaceDocuments, LowerValueRejectionRetainsTypedStageAndCompleteDiagnostics) {
    Preparation failed;
    values::Report lower{values::Status::UnsupportedProfile, 17, 19, 23, true};
    failed.report = detail::NumericalFailure(NumericalStage::ProbeBuild, lower, "source value rejected");
    EXPECT_EQ(failed.report.status, Status::UnsupportedSource);
    EXPECT_EQ(failed.report.numerical_stage, NumericalStage::ProbeBuild);
    EXPECT_EQ(failed.report.numerical.row, 17u);
    EXPECT_EQ(failed.report.numerical.node, 19u);
    const auto document = ResultDocument(failed);
    EXPECT_STREQ(document["numerical_status"].GetString(), "unsupported_profile");
    EXPECT_STREQ(document["numerical_stage"].GetString(), "solid_probe_build");
    EXPECT_EQ(document["representative_row"].GetUint64(), 17u);
    EXPECT_EQ(document["physical_node"].GetUint64(), 19u);
    EXPECT_EQ(document["required_faces"].GetUint64(), 23u);
    EXPECT_TRUE(document["count_complete"].GetBool());
    EXPECT_FALSE(document.HasMember("faces"));
    lower = {values::Status::ResourceLimit};
    const auto capacity = detail::NumericalFailure(NumericalStage::PartBuild, lower, "capacity");
    EXPECT_EQ(capacity.status, Status::ResourceLimit);
}

}
