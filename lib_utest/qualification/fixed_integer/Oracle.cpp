// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Oracle.h"
#include <climits>
#include <stdexcept>
namespace fixed_integer_test {
Integer Input(Big value) {
  Integer result;
  result.negative=value<0;
  if(result.negative) value=-value;
  const Big mask=(Big(1)<<64)-1;
  while(value!=0) {
    if(result.used==8) throw std::invalid_argument("Oracle input exceeds512 bits");
    result.limbs[result.used++]=(value&mask).convert_to<std::uint64_t>();
    value>>=64;
  }
  if(!result.used) result.negative=false;
  return result;
}
Big Value(const Integer& value) {
  Big result=0;
  for(unsigned i=value.used;i;--i) {result<<=64; result+=value.limbs[i-1];}
  return value.negative ? -result : result;
}
std::vector<Case> Corpus() {
  const Big one=1, maximum=(one<<512)-1;
  const std::vector<Big> values{0,1,-1,(one<<63),(one<<64)-1,one<<64,
      (one<<127)-1,-((one<<127)-1),(one<<255)-1,(one<<256)-1,
      one<<448,one<<511,maximum,-maximum};
  std::vector<Case> result;result.reserve(MaximumCases);
  for(const auto& first:values) for(const auto& second:values) {
    for(auto op:{Operation::Add,Operation::Subtract,Operation::Multiply,Operation::Compare})
      result.push_back({op,Input(first),Input(second)});
  }
  for(const auto& value:values) {
    result.push_back({Operation::Negate,Input(value)});
    result.push_back({Operation::Sign,Input(value)});
    for(unsigned shift:{0u,1u,63u,64u,65u,127u,448u,511u,512u,UINT_MAX})
      result.push_back({Operation::Shift,Input(value),{},0,shift});
  }
  for(std::uint64_t scalar:{std::uint64_t{0},std::uint64_t{1},std::uint64_t{1}<<53,
      (std::uint64_t{1}<<53)-1,UINT64_MAX})
    result.push_back({Operation::Scalar,{},{},scalar});
  for(auto op:{Operation::Add,Operation::Subtract,Operation::Multiply,
      Operation::Compare,Operation::Negate,Operation::Sign,Operation::Shift}) {
    for(const auto& value:std::vector<Big>{0,1,-1,maximum}) {
      auto bad=Input(value);bad.overflow=true;
      result.push_back({op,bad,Input(-value),0,1});
      if(op==Operation::Add || op==Operation::Subtract || op==Operation::Multiply || op==Operation::Compare)
        result.push_back({op,Input(-value),bad});
    }
  }
  std::uint64_t state=0xd1b54a32d192ed03ull;
  for(unsigned row=0;row<64;++row) {
    Big a=0,b=0;
    for(unsigned limb=0;limb<8;++limb) {
      state^=state<<13;state^=state>>7;state^=state<<17;
      a=(a<<64)+state;
      state^=state<<13;state^=state>>7;state^=state<<17;
      b=(b<<64)+state;
    }
    a>>=row%5*64;b>>=row%7*64;
    if(row&1)a=-a;if(row&2)b=-b;
    for(auto op:{Operation::Add,Operation::Subtract,Operation::Multiply,Operation::Compare})
      result.push_back({op,Input(a),Input(b)});
  }
  if(result.size()!=1257 || result.size()>MaximumCases)
    throw std::logic_error("Bounded primitive corpus cardinality changed");
  return result;
}
void Same(const Result& a,const Result& b) {
  EXPECT_EQ(a.value.used,b.value.used);EXPECT_EQ(a.value.negative,b.value.negative);
  EXPECT_EQ(a.value.overflow,b.value.overflow);
  for(unsigned i=0;i<8;++i)EXPECT_EQ(a.value.limbs[i],b.value.limbs[i]);
  EXPECT_EQ(a.sign.value,b.sign.value);EXPECT_EQ(a.sign.valid,b.sign.valid);
}
void CheckOracle(const Case& row,const Result& result) {
  const Big a=Value(row.a),b=Value(row.b), maximum=(Big(1)<<512)-1;
  bool invalid=row.a.overflow;
  if(row.operation==Operation::Compare) {
    invalid=invalid||row.b.overflow;
    EXPECT_EQ(result.sign.valid,!invalid);
    EXPECT_EQ(result.sign.value,invalid?0:(a<b?-1:(a>b?1:0)));return;
  }
  if(row.operation==Operation::Sign) {
    EXPECT_EQ(result.sign.valid,!invalid);
    EXPECT_EQ(result.sign.value,invalid?0:(a<0?-1:(a>0?1:0)));return;
  }
  Big expected=0;
  switch(row.operation) {
    case Operation::Add:expected=a+b;invalid=invalid||row.b.overflow;break;
    case Operation::Subtract:expected=a-b;invalid=invalid||row.b.overflow;break;
    case Operation::Multiply:expected=a*b;invalid=invalid||row.b.overflow;break;
    case Operation::Negate:expected=-a;break;
    case Operation::Scalar:expected=row.scalar;invalid=false;break;
    case Operation::Shift:
      if(row.shift>=512 && a!=0) invalid=true;
      else if(a!=0) expected=a*(Big(1)<<row.shift);
      break;
    default:FAIL()<<"Unsupported oracle operation";return;
  }
  invalid=invalid || expected>maximum || expected < -maximum;
  EXPECT_EQ(result.value.overflow,invalid);EXPECT_EQ(result.sign.valid,!invalid);
  ASSERT_LE(result.value.used,8u);
  // Overflow payloads preserve the original implementation, which need not
  // normalize a discarded high carry. Only their invalid status is meaningful.
  if(!invalid) {
    EXPECT_FALSE(!result.value.used && result.value.negative);
    if(result.value.used)EXPECT_NE(result.value.limbs[result.value.used-1],0u);
    for(unsigned i=result.value.used;i<8;++i)EXPECT_EQ(result.value.limbs[i],0u);
  }
  if(invalid)EXPECT_EQ(result.sign.value,0);
  else {EXPECT_EQ(Value(result.value),expected);EXPECT_EQ(result.sign.value,expected<0?-1:(expected>0?1:0));}
}
}  // namespace fixed_integer_test
