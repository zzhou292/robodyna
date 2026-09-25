// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Assertions.h"
#include "NewImpactCases.h"
namespace type25_selection_test {
inline void Same(const s::NativeNewImpactResult& a,const s::NativeNewImpactResult& b,bool exact=false) {
  using type25_geometry_test::Number;
  type25_geometry_test::Same(a.history,b.history,exact);
  EXPECT_EQ(a.active,b.active);EXPECT_EQ(a.row_replaced,b.row_replaced);
  EXPECT_EQ(a.scalar_defined,b.scalar_defined);EXPECT_EQ(a.selected_side,b.selected_side);
  EXPECT_EQ(a.selected_subtriangle,b.selected_subtriangle);EXPECT_EQ(a.far,b.far);
  EXPECT_EQ(a.recontact_intersection,b.recontact_intersection);
  EXPECT_EQ(a.source_key.secondary_source_id,b.source_key.secondary_source_id);
  EXPECT_EQ(a.source_key.generation,b.source_key.generation);EXPECT_EQ(a.source_key.history_index,b.source_key.history_index);
  EXPECT_EQ(a.source_key.main_segment,b.source_key.main_segment);EXPECT_EQ(a.source_local_main,b.source_local_main);
  EXPECT_EQ(a.cache.key.secondary_source_id,b.cache.key.secondary_source_id);
  EXPECT_EQ(a.cache.key.generation,b.cache.key.generation);EXPECT_EQ(a.cache.key.history_index,b.cache.key.history_index);
  EXPECT_EQ(a.cache.key.main_segment,b.cache.key.main_segment);EXPECT_EQ(a.cache.occurrence,b.cache.occurrence);
  EXPECT_EQ(a.cache.local_main,b.cache.local_main);
  Number(a.classification_product,b.classification_product,exact);
  if(a.scalar_defined&s::ImpactPenetrationDefined)Number(a.penetration,b.penetration,exact);
  else {Number(a.penetration,0,true);Number(b.penetration,0,true);}
  if(a.scalar_defined&s::ImpactWeightsAndFarDefined){Number(a.lb,b.lb,exact);Number(a.lc,b.lc,exact);}
  else {Number(a.lb,0,true);Number(a.lc,0,true);Number(b.lb,0,true);Number(b.lc,0,true);EXPECT_EQ(a.far,0);EXPECT_EQ(b.far,0);}
  for(unsigned i=0;i<4;++i) {
    const auto& x=a.projection[i];const auto& y=b.projection[i];EXPECT_EQ(x.defined,y.defined);
    Number(x.raw_lb,y.raw_lb,exact);Number(x.raw_lc,y.raw_lc,exact);
    Number(x.clamped_lb,y.clamped_lb,exact);Number(x.clamped_lc,y.clamped_lc,exact);
    Number(x.distance_squared,y.distance_squared,exact);EXPECT_EQ(x.far,0);EXPECT_EQ(y.far,0);
    Number(x.penetration,0,true);Number(y.penetration,0,true);
    const auto& u=a.cache.sector[i];const auto& v=b.cache.sector[i];
    EXPECT_EQ(u.defined,v.defined);EXPECT_EQ(u.far,v.far);Number(u.penetration,v.penetration,exact);
    Number(u.lb,v.lb,exact);Number(u.lc,v.lc,exact);
  }
  const s::ImpactSideValues* first[]{&a.primary,&a.opposite};
  const s::ImpactSideValues* second[]{&b.primary,&b.opposite};
  for(unsigned side=0;side<2;++side) {
    EXPECT_EQ(first[side]->subtriangle,second[side]->subtriangle);
    EXPECT_EQ(first[side]->intersection,second[side]->intersection);
    for(unsigned i=0;i<4;++i) {
      EXPECT_EQ(first[side]->far[i],second[side]->far[i]);
      EXPECT_EQ(first[side]->cylindrical_gap[i],second[side]->cylindrical_gap[i]);
      Number(first[side]->penetration[i],second[side]->penetration[i],exact);
    }
  }
}
inline s::NativeNewImpactResult NewImpactSentinel() {
  s::NativeNewImpactResult out;out.history=BasicNewImpact().prior;
  out.classification_product=31;out.source_local_main=32;out.cache.occurrence=33;
  out.primary.subtriangle=2;out.opposite.far[3]=3;return out;
}
} // namespace type25_selection_test
