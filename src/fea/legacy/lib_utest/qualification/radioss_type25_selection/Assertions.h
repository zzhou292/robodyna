// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Cases.h"
#include "../radioss_type25_local_geometry/Assertions.h"
namespace type25_selection_test {
template<class Result>
inline void SameClassified(const Result& a,const Result& b,bool exact=false) {
  using type25_geometry_test::Number;
  type25_geometry_test::Same(a.history,b.history,exact);
  EXPECT_EQ(a.active,b.active);
  EXPECT_EQ(a.selected_subtriangle,b.selected_subtriangle);
  Number(a.classification_product,b.classification_product,exact);
  Number(a.distance_squared,b.distance_squared,exact);
  EXPECT_EQ(a.cache.key.secondary_source_id,b.cache.key.secondary_source_id);
  EXPECT_EQ(a.cache.key.generation,b.cache.key.generation);
  EXPECT_EQ(a.cache.key.history_index,b.cache.key.history_index);
  EXPECT_EQ(a.cache.key.main_segment,b.cache.key.main_segment);
  EXPECT_EQ(a.cache.occurrence,b.cache.occurrence);EXPECT_EQ(a.cache.local_main,b.cache.local_main);
  for(unsigned i=0;i<4;++i) {
    SCOPED_TRACE(i);const auto& x=a.sector[i];const auto& y=b.sector[i];
    EXPECT_EQ(x.defined,y.defined);EXPECT_EQ(x.far,y.far);
    Number(x.penetration,y.penetration,exact);Number(x.distance_squared,y.distance_squared,exact);
    if(x.defined&s::RawBarycentricDefined) {Number(x.raw_lb,y.raw_lb,exact);Number(x.raw_lc,y.raw_lc,exact);}
    else {Number(x.raw_lb,0,true);Number(x.raw_lc,0,true);Number(y.raw_lb,0,true);Number(y.raw_lc,0,true);}
    if(x.defined&s::ClampedBarycentricDefined) {Number(x.clamped_lb,y.clamped_lb,exact);Number(x.clamped_lc,y.clamped_lc,exact);}
    else {Number(x.clamped_lb,0,true);Number(x.clamped_lc,0,true);Number(y.clamped_lb,0,true);Number(y.clamped_lc,0,true);}
    const auto& u=a.cache.sector[i];const auto& v=b.cache.sector[i];
    EXPECT_EQ(u.defined,v.defined);EXPECT_EQ(u.far,v.far);Number(u.penetration,v.penetration,exact);
    if(u.defined&s::ClampedBarycentricDefined){Number(u.lb,v.lb,exact);Number(u.lc,v.lc,exact);}
    else {Number(u.lb,0,true);Number(u.lc,0,true);Number(v.lb,0,true);Number(v.lc,0,true);}
  }
}
inline void Same(const s::NativeRetainedResult& a,const s::NativeRetainedResult& b,bool exact=false) {
  SameClassified(a,b,exact);EXPECT_EQ(a.prior_subtriangle,b.prior_subtriangle);
}
inline s::NativeRetainedResult Sentinel() {
  s::NativeRetainedResult result;result.history=Basic().prior;
  result.classification_product=31;result.distance_squared=32;result.cache.occurrence=33;
  for(auto& sector:result.sector){sector.penetration=34;sector.distance_squared=35;}
  return result;
}
} // namespace type25_selection_test
