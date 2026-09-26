#include "../Internal.h"
#include "modelio/source_assembly/SourceAssemblyData.h"
#include "lib_src/collision/radioss_type25/source_shells/Values.h"
#include "lib_utest/qualification/radioss_type25_gap_source/NativeOracle.h"
#include "lib_utils/BoundedArena.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
namespace crash::cases::vehicle_self_contact::native::gap_operands::test {
TEST(GapOperandsSource, RawPropertyThicknessKeepsOriginalWorkingBits) {
    modelio::assembly::Section section;
    section.source.keyword="*SECTION_SHELL";section.source_elform=2;section.cards.resize(2);
    section.cards[1].values={2.28};section.thickness_m[0]=2.28*.001;
    EXPECT_TRUE(tl::math::SameScalarBits(detail::PropertyThickness(section,{.001,1000.,1.}),2.28));
    section.thickness_m[0]=std::nextafter(section.thickness_m[0],1.);
    EXPECT_THROW(detail::PropertyThickness(section,{.001,1000.,1.}),std::exception);
    section.thickness_m[0]=2.28*.001;section.source.keyword="*SECTION_SHELL_COMPOSITE";
    EXPECT_THROW(detail::PropertyThickness(section,{.001,1000.,1.}),std::exception);
    section.source.keyword="*SECTION_SHELL";section.cards[1].values[0].reset();
    EXPECT_THROW(detail::PropertyThickness(section,{.001,1000.,1.}),std::exception);
}
TEST(GapOperandsSource, UnknownOverrideWritersCannotBecomeProvenZero) {
    for(const auto* keyword:{"*PART_CONTACT","*ELEMENT_SHELL_THICKNESS","*INITIAL_THICKNESS_SHELL","*INCLUDE_RADIOSS"}) {
        output::Document inventory;
        const auto text=std::string("{\"source\":{\"keyword_counts\":{\"")+keyword+"\":1}}}";
        inventory.Parse(text.c_str());
        ASSERT_FALSE(inventory.HasParseError());
        EXPECT_THROW(detail::CheckKeywordInventory(inventory),detail::Failure);
    }
    output::Document inventory;
    inventory.Parse(R"({"source":{"keyword_counts":{"*PART":2,"*ELEMENT_SHELL":3,"*CONSTRAINED_SPOTWELD_ID":1}}})");
    EXPECT_EQ(detail::CheckKeywordInventory(inventory),6u);
}
TEST(GapOperandsSource, MaxCertificateRequiresFiniteValuesAndOnlyPositiveZero) {
    for(double value:{0.,1.,std::numeric_limits<double>::denorm_min(),std::numeric_limits<double>::max()}) {
        EXPECT_NO_THROW(detail::CheckMaximumTerm(value));
    }
    for(double value:{-0.,-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
        EXPECT_THROW(detail::CheckMaximumTerm(value),detail::Failure);
    }
}
TEST(GapOperandsSource, CertifiedLayerPermutationMatchesOriginalGapController) {
    values::PhysicalShell shells[3];
    for(unsigned i=0;i<3;++i) {
        shells[i].source_element_id=100+i;shells[i].layout=n::ShellLayout::Quad4;
        for(unsigned k=0;k<4;++k)shells[i].nodes[k]=k;
        shells[i].property_thickness=i==0?.5:2.28;
        detail::CheckMaximumTerm(n::source_shells::detail::HalfGap(shells[i],0));
    }
    n::startup::Main main;main.source_id=100;main.segment_type=2;
    for(unsigned k=0;k<4;++k)main.nodes[k]=k;
    const std::uint32_t roster[]{0,1,2,3};
    values::Input input;input.profile={1,0,1,1,0,0,1.,1e30,1e30};input.node_count=4;
    input.shells=shells;input.shell_count=3;input.mains=&main;input.main_count=input.primary_count=1;
    input.secondary_nodes=roster;input.secondary_count=4;input.main_nodes=roster;input.main_node_count=4;
    const auto reference=gap_source_test::Oracle(input);
    std::swap(shells[0],shells[2]);
    const auto permuted=gap_source_test::Oracle(input);
    EXPECT_EQ(reference.secondary,permuted.secondary);EXPECT_EQ(reference.main_nodes,permuted.main_nodes);
    values::Forecast forecast;
    ASSERT_EQ(values::Preflight(input,{},forecast).status,values::Status::Ok);
    tl::util::HostArena scratch;
    ASSERT_TRUE(scratch.Initialize(forecast.scratch_bytes));
    double secondary[4],main_nodes[4];values::MainGapFields mains[1];
    ASSERT_EQ(values::Build(input,{},scratch.data(),scratch.bytes(),{secondary,4,main_nodes,4,mains,1}).status,values::Status::Ok);
    for(unsigned i=0;i<4;++i) {
        EXPECT_TRUE(tl::math::SameScalarBits(secondary[i],reference.secondary[i]));
        EXPECT_TRUE(tl::math::SameScalarBits(main_nodes[i],reference.main_nodes[i]));
    }
}
TEST(GapOperandsSource, OperandDigestBindsSourceUnitsFamiliesAndRawValues) {
    detail::Packed p;p.counts.nodes=2;p.proof.no_retained_trusses=true;
    p.beams.push_back({101,{0,1},0.,4.});p.bindings.push_back({Family::Beam18,101,101,10,0,0});
    Provenance source;source.units={.001,1000.,1.};
    source.source_digest=source.contributor_digest=source.property_digest=std::string(64,'a');
    const auto digest=detail::Digest(p,source,1u<<20);
    auto copy=p;
    EXPECT_EQ(detail::Digest(copy,source,1u<<20),digest);
    copy.beams[0].native_area=std::nextafter(4.,5.);
    EXPECT_NE(detail::Digest(copy,source,1u<<20),digest);
    copy=p;copy.bindings[0].native_id=102;
    EXPECT_NE(detail::Digest(copy,source,1u<<20),digest);
    copy=p;copy.proof.maximum_terms=1;
    EXPECT_NE(detail::Digest(copy,source,1u<<20),digest);
    EXPECT_THROW(detail::Digest(p,source,1),std::exception);
}
}
