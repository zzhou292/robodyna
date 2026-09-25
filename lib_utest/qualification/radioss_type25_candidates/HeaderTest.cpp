#include "lib_src/collision/radioss_type25/candidates/Packing.h"
#include <gtest/gtest.h>
namespace c=tlfea::contact::radioss_type25::candidates;
TEST(NativeCandidateConsumer,IndependentProductionValueLink) {
  c::LocalRow row;row.secondary_node=5;row.main_count=1;row.segment_type=1;
  row.nodes[0]=1;row.nodes[1]=2;row.nodes[2]=3;row.nodes[3]=3;
  row.screen.vertices[0]={-1,0,0};row.screen.vertices[1]={1,0,0};
  row.screen.vertices[2]={0,1,0};row.screen.vertices[3]={0,1,0};
  row.screen.secondary={0,.5,.125};row.screen.secondary_gap=.25;row.screen.margin=.125;
  c::FilterResult result;ASSERT_EQ(c::EvaluateLocal(row,&result),c::Status::Ok);
  EXPECT_TRUE(result.included);EXPECT_GT(result.squared_clearance,0.);
}
