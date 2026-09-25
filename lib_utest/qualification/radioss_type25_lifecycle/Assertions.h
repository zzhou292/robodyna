// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"
#include "../radioss_type25_local_geometry/Assertions.h"
namespace type25_lifecycle_test {
template<class A,class B> inline void Same(const A& a,const B& b,bool exact=false) {
  using type25_geometry_test::Number;
  ASSERT_EQ(a.rows.size(),b.rows.size());ASSERT_EQ(a.occurrences.size(),b.occurrences.size());
  ASSERT_EQ(a.geometry.size(),b.geometry.size());
  for(std::size_t i=0;i<a.rows.size();++i) {
    const auto& x=a.rows[i];const auto& y=b.rows[i];type25_geometry_test::Same(x.history,y.history,exact);
    EXPECT_EQ(x.initial_contact_flag,y.initial_contact_flag);
    for(unsigned j=0;j<4;++j)EXPECT_EQ(x.sliding_reference[j],y.sliding_reference[j]);
    EXPECT_EQ(x.retained_count,y.retained_count);EXPECT_EQ(x.optimized_count,y.optimized_count);
    EXPECT_EQ(x.sliding_count,y.sliding_count);EXPECT_EQ(x.continuation_count,y.continuation_count);
    EXPECT_EQ(x.new_impact_count,y.new_impact_count);EXPECT_EQ(x.kept_count,y.kept_count);
    EXPECT_EQ(x.zero_sum_resets,y.zero_sum_resets);
  }
  for(std::size_t i=0;i<a.occurrences.size();++i) {
    SCOPED_TRACE(i);const auto& x=a.occurrences[i];const auto& y=b.occurrences[i];
    EXPECT_EQ(x.cache_initialized,y.cache_initialized);EXPECT_EQ(x.secondary,y.secondary);
    EXPECT_EQ(x.local_main,y.local_main);EXPECT_EQ(x.origin,y.origin);EXPECT_EQ(x.source_ordinal,y.source_ordinal);
    EXPECT_EQ(x.cache.key.secondary_source_id,y.cache.key.secondary_source_id);
    EXPECT_EQ(x.cache.key.generation,y.cache.key.generation);EXPECT_EQ(x.cache.key.history_index,y.cache.key.history_index);
    EXPECT_EQ(x.cache.key.main_segment,y.cache.key.main_segment);EXPECT_EQ(x.cache.local_main,y.cache.local_main);
    EXPECT_EQ(x.cache.occurrence,y.cache.occurrence);EXPECT_EQ(x.cache.occurrence,i);
    for(unsigned j=0;j<4;++j) {
      const auto& u=x.cache.sector[j];const auto& v=y.cache.sector[j];
      EXPECT_EQ(u.defined,v.defined);EXPECT_EQ(u.far,v.far);Number(u.penetration,v.penetration,exact);
      if(u.defined&n::selection::ClampedBarycentricDefined){Number(u.lb,v.lb,exact);Number(u.lc,v.lc,exact);}
      else {Number(u.lb,0,true);Number(u.lc,0,true);Number(v.lb,0,true);Number(v.lc,0,true);}
    }
    const auto& u=x.selected;const auto& v=y.selected;EXPECT_EQ(u.enabled,v.enabled);
    EXPECT_EQ(u.local_main,v.local_main);EXPECT_EQ(u.subtriangle,v.subtriangle);EXPECT_EQ(u.selection_code,v.selection_code);
    EXPECT_EQ(u.key.secondary_source_id,v.key.secondary_source_id);EXPECT_EQ(u.key.generation,v.key.generation);
    EXPECT_EQ(u.key.history_index,v.key.history_index);EXPECT_EQ(u.key.main_segment,v.key.main_segment);
    Number(u.lb,v.lb,exact);Number(u.lc,v.lc,exact);Number(u.incoming_stiffness,v.incoming_stiffness,exact);
    if(u.enabled)type25_geometry_test::Same(a.geometry[i],b.geometry[i],exact);
  }
}
inline l::HostResult RunLifecycleFixture(Fixture& fixture) {
  l::HostResult out;const auto report=l::EvaluateNativeLifecycleHost(fixture.Input(),Fixture::Limits(),&out);
  EXPECT_EQ(report.status,n::selection::Status::Ok);return out;
}
} // namespace type25_lifecycle_test
