// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Oracle.h"
#include <climits>
#include <type_traits>
namespace fixed_integer_test {
TEST(FixedInteger, CompleteBoundedCorpusMatchesUnboundedBoostOracle) {
  const auto rows=Corpus();ASSERT_EQ(rows.size(),1257u);
  for(std::size_t i=0;i<rows.size();++i) {SCOPED_TRACE(i);CheckOracle(rows[i],Evaluate(rows[i]));}
}
TEST(FixedInteger, ScalarInterpolationWeightsKeepEveryUnsignedBitAndZeroPadding) {
  for(std::uint64_t value:{std::uint64_t{0},std::uint64_t{1},std::uint64_t{1}<<53,
      (std::uint64_t{1}<<53)-1,UINT64_MAX}) {
    const auto actual=Arithmetic::FromU64(value);
    EXPECT_EQ(Value(actual),Big(value));EXPECT_FALSE(actual.negative||actual.overflow);
    for(unsigned i=1;i<8;++i)EXPECT_EQ(actual.limbs[i],0u);
  }
  const auto one=Arithmetic::FromU64(1),zero=Arithmetic::Subtract(one,one);
  EXPECT_EQ(zero.used,0u);EXPECT_FALSE(Arithmetic::Negate(zero).negative);
}
TEST(FixedInteger, CarryBorrowCancellationAndNegativeOrderingRemainExact) {
  const Big high=Big(1)<<448;
  const auto carry=Arithmetic::Add(Input(high-1),Arithmetic::FromU64(1));
  EXPECT_EQ(Value(carry),high);EXPECT_EQ(carry.used,8u);
  const auto borrow=Arithmetic::Subtract(Input(high),Arithmetic::FromU64(1));
  EXPECT_EQ(Value(borrow),high-1);EXPECT_EQ(borrow.used,7u);
  const auto cancelled=Arithmetic::Add(Input(-high),Input(high));
  EXPECT_EQ(cancelled.used,0u);EXPECT_FALSE(cancelled.negative||cancelled.overflow);
  const auto order=Arithmetic::Compare(Input(-high),Input(-1));
  EXPECT_TRUE(order.valid);EXPECT_EQ(order.value,-1);
}
TEST(FixedInteger, LeftShiftPreservesSignedMagnitudeAndRejectsEveryLostHighBit) {
  for(const Big value:std::vector<Big>{0,1,-1,(Big(1)<<64)-1,Big(1)<<511})
    for(unsigned amount:{0u,1u,63u,64u,65u,127u,448u,511u,512u,UINT_MAX}) {
      const Case row{Operation::Shift,Input(value),{},0,amount};CheckOracle(row,Evaluate(row));
    }
  const auto exact=Arithmetic::ShiftLeft(Input(-1),511);
  EXPECT_EQ(Value(exact),-(Big(1)<<511));EXPECT_FALSE(exact.overflow);
  EXPECT_TRUE(Arithmetic::ShiftLeft(Input(1),512).overflow);
}
TEST(FixedInteger, OverflowCannotDisappearThroughCancellationZeroOrComparison) {
  auto bad=Arithmetic::Add(Input((Big(1)<<512)-1),Input(1));ASSERT_TRUE(bad.overflow);
  EXPECT_TRUE(Arithmetic::Multiply(bad,{}).overflow);
  EXPECT_TRUE(Arithmetic::Subtract(bad,bad).overflow);
  EXPECT_TRUE(Arithmetic::Negate(bad).overflow);
  EXPECT_TRUE(Arithmetic::ShiftLeft(bad,0).overflow);
  EXPECT_FALSE(Arithmetic::Compare(bad,bad).valid);
  EXPECT_FALSE(Arithmetic::Result(bad).valid);
  auto bad_zero=Arithmetic::FromU64(0);bad_zero.overflow=true;
  EXPECT_TRUE(Arithmetic::ShiftLeft(bad_zero,UINT_MAX).overflow);
}
TEST(FixedInteger, SharedVectorBodiesAndIntegerLayoutRemainBounded) {
  static_assert(std::is_trivially_copyable_v<Integer>);
  static_assert(std::is_standard_layout_v<Integer>);
  static_assert(sizeof(Integer)<=80);
  Arithmetic::Integer3 a{Input(-3),Input(4),Input(5)},b{Input(7),Input(-2),Input(3)};
  EXPECT_EQ(Value(Arithmetic::Dot(a,b)),Big(-14));
  const auto d=Arithmetic::Subtract(a,b);
  EXPECT_EQ(Value(d.x),-10);EXPECT_EQ(Value(d.y),6);EXPECT_EQ(Value(d.z),2);
}
}  // namespace fixed_integer_test
