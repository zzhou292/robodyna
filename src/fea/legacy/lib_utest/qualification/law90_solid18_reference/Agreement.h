// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
#include "WorkingModeBound.h"
#include "lib_utest/qualification/solid18_reference/NativeOracle.h"
#include <gtest/gtest.h>
#include <iomanip>
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
template<std::size_t N>
inline ::testing::AssertionResult GroupDetails(const std::array<double,N>& a,
    const std::array<double,N>& b,unsigned first,unsigned count,double relative,
    bool norm=true) {
  double scale=0;
  for(unsigned i=first;i<first+count;++i)
    scale=std::max({scale,std::abs(a[i]),std::abs(b[i])});
  for(unsigned i=first;i<first+count;++i) {
    const double bound=norm ? (relative+64*std::numeric_limits<double>::epsilon())*scale :
        relative*std::max(std::abs(a[i]),std::abs(b[i]))+64*std::numeric_limits<double>::epsilon()*scale;
    if(!std::isfinite(a[i])||!std::isfinite(b[i])||std::abs(a[i]-b[i])>bound)
      return ::testing::AssertionFailure()<<std::setprecision(17)<<"index="<<i
          <<" actual="<<a[i]<<" native="<<b[i]<<" delta="<<a[i]-b[i]
          <<" bound="<<bound<<" group_scale="<<scale;
  }
  return ::testing::AssertionSuccess();
}
inline ::testing::AssertionResult ReferenceAgreement(const std::array<double,ReferenceCount>& a,
    const std::array<double,ReferenceCount>& b,double conditioning=0) {
  std::array<double,solid18_test::ValueCount> av{},bv{};
  std::copy_n(a.begin(),av.size(),av.begin());
  std::copy_n(b.begin(),bv.size(),bv.begin());
  // Only the 12 higher-mode values have a separate operation-derived working
  // bound below. Every other base field retains its owning comparator unchanged.
  if(conditioning>0)std::copy_n(bv.begin()+42,12,av.begin()+42);
  const bool base=conditioning>0 ? solid18_test::AgreeWorkingUnits(av,bv,conditioning)
                               : solid18_test::Agree(av,bv);
  const double relative=conditioning>0 ?
      256*std::numeric_limits<double>::epsilon()*std::max(1.0,conditioning) : 2e-11;
  if(!base) {
    // Diagnostics repeat only the owning comparator's existing group schedule;
    // the existing Agree/AgreeWorkingUnits result remains admission authority.
    for(const auto range:{std::pair<unsigned,unsigned>{0,9},{9,24},{33,9},{42,12}}) {
      if(conditioning>0&&range.first==42)continue;
      const auto result=GroupDetails(a,b,range.first,range.second,relative,
                                    conditioning>0&&range.first>=33);
      if(!result)return result;
    }
    for(unsigned ip=0;ip<8;++ip) {
      auto result=GroupDetails(a,b,54+10*ip,9,relative,conditioning>0);
      if(!result)return result;
      result=GroupDetails(a,b,63+10*ip,1,relative,false);
      if(!result)return result;
    }
    for(const auto range:{std::pair<unsigned,unsigned>{134,2},{136,1},{137,1},{138,9},{147,1}}) {
      const auto result=GroupDetails(a,b,range.first,range.second,relative,false);
      if(!result)return result;
    }
    return ::testing::AssertionFailure()<<"owning base comparator failed";
  }
  if(conditioning>0) {
    for(unsigned mode=0;mode<4;++mode)for(unsigned axis=0;axis<3;++axis) {
      const unsigned i=42+3*mode+axis;
      const auto bound=HigherModeWorkingBound(a,b,mode,axis);
      if(!std::isfinite(a[i])||!std::isfinite(b[i])||
         std::abs(static_cast<long double>(a[i])-b[i])>bound)
        return ::testing::AssertionFailure()<<std::setprecision(17)<<"working higher-mode index="<<i
            <<" actual="<<a[i]<<" native="<<b[i]<<" delta="<<a[i]-b[i]<<" bound="<<bound;
    }
  }
  const auto group=[&](unsigned first,unsigned count) {
    return GroupDetails(a,b,first,count,relative);
  };
  for(const auto range:{std::pair<unsigned,unsigned>{148,9},{157,1}}) {
    const auto result=group(range.first,range.second);if(!result)return result;
  }
  for(unsigned ip=0;ip<8;++ip) {
    const auto result=group(158+72*ip,72);if(!result)return result;
  }
  return group(734,21);
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
    return GroupDetails(a,b,begin,count,2e-11);
  };
  for(unsigned count:{9u,24u,24u,12u,1u,1u}) {
    const auto result=group(count);
    if(!result) return ::testing::AssertionFailure()<<"current geometry group ending "<<cursor<<": "<<result.message();
  }
  for(unsigned ip=0;ip<8;++ip) {
    for(unsigned count:{1u,9u,24u,48u}) {
      const auto result=group(count);
      if(!result) return ::testing::AssertionFailure()<<"current point "<<ip<<" group ending "<<cursor<<": "<<result.message();
    }
  }
  const auto displacement=group(24);
  if(!displacement)return ::testing::AssertionFailure()<<"reference displacement: "<<displacement.message();
  for(unsigned ip=0;ip<8;++ip) {
    for(unsigned count:{9u,9u,6u,6u}) {
      const auto result=group(count);
      if(!result) return ::testing::AssertionFailure()<<"total point "<<ip<<" group ending "<<cursor<<": "<<result.message();
    }
  }
  return cursor==CurrentCount ? ::testing::AssertionSuccess() : ::testing::AssertionFailure()<<"packet shape";
}
}
