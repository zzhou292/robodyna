#include "../Internal.h"
#include "modelio/source_assembly/SourceLines.h"
#include <gtest/gtest.h>
#include <iomanip>
#include <sstream>
namespace crash::modelio::beam18::test {
using namespace assembly::reader;
namespace {
std::string Card(std::initializer_list<const char*> values,unsigned width=10){std::ostringstream s;for(auto v:values)s<<std::setw(width)<<v;return s.str();}
}
TEST(SourceFieldsShared, SignedZeroBlankExplicitZeroAndMalformedValuesStayDistinct) {
    EXPECT_FALSE(SourceScalar(Card({""}),0));
    EXPECT_EQ(output::Bits(RequiredScalar(Card({"-0.0"}),0)),output::Bits(-0.0));
    EXPECT_EQ(RequiredScalar(Card({"+1.25"}),0),1.25);
    EXPECT_THROW(RequiredScalar(Card({"nan"}),0),std::runtime_error);
    EXPECT_THROW(RequiredScalar(Card({"1junk"}),0),std::runtime_error);
    EXPECT_THROW(RequireBlankFields(Card({"0"}),0,1),std::runtime_error);
    EXPECT_THROW(RequireBlankFields("",2,1),std::runtime_error);
}
TEST(SourceFieldsShared, CurvesKeepOwnedBitsAndLateFailurePreservesBothOutputs) {
    assembly::SourceBlock block;block.keyword="*DEFINE_CURVE";
    std::vector<std::pair<std::size_t,std::string>> rows{{1,Card({"42","0","1","1"})},
        {2,Card({"-0.0","1.25"},20)},{3,Card({"1","2.5"},20)}};
    std::vector<double> x,y;
    ReadSourceCurve(block,rows,42,2,1e6,x,y);
    EXPECT_EQ(output::Bits(x[0]),output::Bits(-0.0));EXPECT_EQ(y[1],2.5e6);
    const auto old_x=x,old_y=y;
    EXPECT_NO_THROW(ReadSourceCurve(block,rows,42,2,1e6,x,y));
    rows.back().second=Card({"1","3"},20);
    EXPECT_THROW(ReadSourceCurve(block,rows,42,2,1e6,x,y),std::runtime_error);
    EXPECT_EQ(x,old_x);EXPECT_EQ(y,old_y);
    x.clear();y.clear();rows.back().second=Card({"1","nan"},20);
    EXPECT_THROW(ReadSourceCurve(block,rows,42,2,1e6,x,y),std::runtime_error);
    EXPECT_TRUE(x.empty());EXPECT_TRUE(y.empty());
}
TEST(SourceFieldsShared, RequestedLinesPreserveColumnsKeywordAndCoverage) {
    const std::string text="*NODE\n$ comment\n       1      2 $ note\n*ELEMENT_BEAM\n       3\n";
    std::vector<std::string> keys,cards;
    VisitSourceLines(text,{3,5},[&](auto,const auto& key,const auto& row){keys.push_back(key);cards.push_back(row);});
    EXPECT_EQ(keys,(std::vector<std::string>{"*NODE","*ELEMENT_BEAM"}));
    EXPECT_EQ(cards[0],"       1      2");
    const auto noop=[](auto,const auto&,const auto&){};
    EXPECT_THROW(VisitSourceLines(text,{3,3},noop),std::runtime_error);
    EXPECT_THROW(VisitSourceLines(text,{6},noop),std::runtime_error);
}
} // namespace crash::modelio::beam18::test
