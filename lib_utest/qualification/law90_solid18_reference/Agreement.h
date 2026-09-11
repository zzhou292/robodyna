// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
#include "lib_utest/qualification/solid18_reference/NativeOracle.h"
#include <gtest/gtest.h>
namespace law90_reference_test {
template<std::size_t N>
inline bool GroupAgreement(const std::array<double,N>& a,const std::array<double,N>& b,
                           unsigned first,unsigned count,double relative) {
  double scale=0;
  for(unsigned i=first;i<first+count;++i) {
    if(!std::isfinite(a[i]) || !std::isfinite(b[i])) return false;
    scale=std::max({scale,std::abs(a[i]),std::abs(b[i])});
  }
  const double bound=(relative+64*std::numeric_limits<double>::epsilon())*scale;
  for(unsigned i=first;i<first+count;++i) if(std::abs(a[i]-b[i])>bound) return false;
  return true;
}
inline bool ReferenceAgreement(const std::array<double,ReferenceCount>& a,
    const std::array<double,ReferenceCount>& b,double conditioning=0) {
  std::array<double,solid18_test::ValueCount> av{},bv{};
  std::copy_n(a.begin(),av.size(),av.begin());
  std::copy_n(b.begin(),bv.size(),bv.begin());
  const bool base=conditioning>0 ? solid18_test::AgreeWorkingUnits(av,bv,conditioning)
                               : solid18_test::Agree(av,bv);
  if(!base) return false;
  const double relative=conditioning>0 ?
      256*std::numeric_limits<double>::epsilon()*std::max(1.0,conditioning) : 2e-11;
  if(!GroupAgreement(a,b,148,9,relative) || !GroupAgreement(a,b,157,1,relative)) return false;
  for(unsigned ip=0;ip<8;++ip)
    if(!GroupAgreement(a,b,158+72*ip,72,relative)) return false;
  return GroupAgreement(a,b,734,21,relative);
}
inline std::array<double,ReferenceCount> WorkingReferenceToSI(
    const std::array<double,ReferenceCount>& input) {
  std::array<double,solid18_test::ValueCount> base{};
  std::copy_n(input.begin(),base.size(),base.begin());
  base=solid18_test::NativeWorkingToSI(base);
  auto result=input;
  std::copy(base.begin(),base.end(),result.begin());
  for(unsigned i=148;i<157;++i) result[i]*=1000;
  result[157]*=1e-9;
  for(unsigned i=158;i<734;++i) result[i]*=1000;
  for(unsigned i=734;i<755;++i) result[i]*=.001;
  return result;
}
inline ::testing::AssertionResult CurrentAgreement(const std::array<double,CurrentCount>& a,
    const std::array<double,CurrentCount>& b) {
  unsigned cursor=0;
  const auto group=[&](unsigned count) {
    const unsigned begin=cursor;
    cursor+=count;
    return GroupAgreement(a,b,begin,count,2e-11);
  };
  for(unsigned count:{9u,24u,24u,12u,1u,1u}) {
    if(!group(count)) return ::testing::AssertionFailure()<<"current geometry group ending "<<cursor;
  }
  for(unsigned ip=0;ip<8;++ip) {
    for(unsigned count:{1u,9u,24u,48u}) {
      if(!group(count)) return ::testing::AssertionFailure()<<"current point "<<ip<<" group ending "<<cursor;
    }
  }
  if(!group(24)) return ::testing::AssertionFailure()<<"reference displacement";
  for(unsigned ip=0;ip<8;++ip) {
    for(unsigned count:{9u,9u,6u,6u}) {
      if(!group(count)) return ::testing::AssertionFailure()<<"total point "<<ip<<" group ending "<<cursor;
    }
  }
  return cursor==CurrentCount ? ::testing::AssertionSuccess() : ::testing::AssertionFailure()<<"packet shape";
}
}
