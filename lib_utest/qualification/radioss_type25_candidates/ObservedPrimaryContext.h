#pragma once
#include "PackingFixture.h"
#include "PrimaryContextFixture.h"
// Immutable qualification fixture from observation2; no production data source.
namespace candidate_test {
struct ObservedPrimaryRow {c::LocalRow row;int worker_count=0,unshifted_role=0;};
inline std::vector<ObservedPrimaryRow> ObservedPrimaryRows() {
  const double xyz[]={-0x1.4000000000000p+2,-0x1.4000000000000p+2,0x1.e666666666666p+0,0x0.0p+0,-0x1.4000000000000p+2,0x1.0000000000000p+1,0x0.0p+0,0x0.0p+0,0x1.0000000000000p+1,-0x1.4000000000000p+2,0x0.0p+0,0x1.e666666666666p+0,0x1.4000000000000p+2,-0x1.4000000000000p+2,0x1.0cccccccccccdp+1,0x1.4000000000000p+2,0x0.0p+0,0x1.0cccccccccccdp+1,0x0.0p+0,0x1.4000000000000p+2,0x1.0000000000000p+1,-0x1.4000000000000p+2,0x1.4000000000000p+2,0x1.e666666666666p+0,0x1.4000000000000p+2,0x1.4000000000000p+2,0x1.0cccccccccccdp+1,-0x1.4000000000000p+4,-0x1.4000000000000p+4,0x0.0p+0,0x0.0p+0,-0x1.4000000000000p+4,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,-0x1.4000000000000p+4,0x0.0p+0,0x0.0p+0,0x1.4000000000000p+4,-0x1.4000000000000p+4,0x0.0p+0,0x1.4000000000000p+4,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x1.4000000000000p+4,0x0.0p+0,-0x1.4000000000000p+4,0x1.4000000000000p+4,0x0.0p+0,0x1.4000000000000p+4,0x1.4000000000000p+4,0x0.0p+0};
  const double velocity[]={0x1.f400000000000p+9,0x0.0p+0,-0x1.3880000000000p+13,0x1.f400000000000p+9,0x0.0p+0,-0x1.3880000000000p+13,0x1.f400000000000p+9,0x0.0p+0,-0x1.3880000000000p+13,0x1.f400000000000p+9,0x0.0p+0,-0x1.3880000000000p+13,0x1.f400000000000p+9,0x0.0p+0,-0x1.3880000000000p+13,0x1.f400000000000p+9,0x0.0p+0,-0x1.3880000000000p+13,0x1.f400000000000p+9,0x0.0p+0,-0x1.3880000000000p+13,0x1.f400000000000p+9,0x0.0p+0,-0x1.3880000000000p+13,0x1.f400000000000p+9,0x0.0p+0,-0x1.3880000000000p+13,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0};
  const int code[]={0,0,0,0,0,0,0,0,0,7,7,7,7,7,7,7,7,7};
  const int secondary[]={10,11,14,13,12,15,17,16,18,1,2,5,4,3,6,8,7,9};
  const double secondary_gap[]={0x1.0000000000000p-1,0x1.0000000000000p-1,0x1.0000000000000p-1,0x1.0000000000000p-1,0x1.0000000000000p-1,0x1.0000000000000p-1,0x1.0000000000000p-1,0x1.0000000000000p-1,0x1.0000000000000p-1,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0,0x0.0p+0};
  const int roles[]={9,10,11,12,13,14,15,16,-1,-2,-3,-4,-5,-6,-7,-8};
  std::vector<ObservedPrimaryRow> result;
  {
  const int nodes[]={10,11,12,12,10,12,13,13};
  const double main_gap[]={0x1.0000000000000p-1,0x1.0000000000000p-1};
  const double curvature[]={0x0.0p+0,0x0.0p+0};
    for(unsigned m=0;m<2;++m)for(unsigned secondary_row=0;secondary_row<18;++secondary_row) {
      c::LocalRow row;const auto slave=secondary[secondary_row];bool own=false;
      for(unsigned j=0;j<4;++j)if(nodes[4*m+j]==slave)own=true;
      if(own)continue; // Native TRIVOX source exclusion precedes the packet.
      row.secondary_node=slave;row.main_count=8;row.segment_type=roles[0+m];
      row.constraint_codes[4]=code[slave-1];row.previous_dt=0.;
      row.screen.secondary={xyz[3*(slave-1)],xyz[3*(slave-1)+1],xyz[3*(slave-1)+2]};
      row.secondary_velocity={velocity[3*(slave-1)],velocity[3*(slave-1)+1],velocity[3*(slave-1)+2]};
      row.screen.secondary_gap=secondary_gap[secondary_row];row.screen.main_gap=main_gap[m];
      row.screen.curvature=curvature[m];row.screen.margin=0x1.b504f3a13b1b3p+1;
      row.screen.drad=0x0.0p+0;row.screen.gap_load=0x0.0p+0;
      row.screen.stored_motion=0x0.0p+0;
      for(unsigned j=0;j<4;++j){const auto node=nodes[4*m+j];row.nodes[j]=node;row.constraint_codes[j]=code[node-1];
        row.screen.vertices[j]={xyz[3*(node-1)],xyz[3*(node-1)+1],xyz[3*(node-1)+2]};
        row.main_velocities[j]={velocity[3*(node-1)],velocity[3*(node-1)+1],velocity[3*(node-1)+2]};}
      result.push_back({row,2,roles[m]});
    }
  }
  {
  const int nodes[]={11,14,15,15,11,15,12,12};
  const double main_gap[]={0x1.0000000000000p-1,0x1.0000000000000p-1};
  const double curvature[]={0x0.0p+0,0x0.0p+0};
    for(unsigned m=0;m<2;++m)for(unsigned secondary_row=0;secondary_row<18;++secondary_row) {
      c::LocalRow row;const auto slave=secondary[secondary_row];bool own=false;
      for(unsigned j=0;j<4;++j)if(nodes[4*m+j]==slave)own=true;
      if(own)continue; // Native TRIVOX source exclusion precedes the packet.
      row.secondary_node=slave;row.main_count=8;row.segment_type=roles[2+m];
      row.constraint_codes[4]=code[slave-1];row.previous_dt=0.;
      row.screen.secondary={xyz[3*(slave-1)],xyz[3*(slave-1)+1],xyz[3*(slave-1)+2]};
      row.secondary_velocity={velocity[3*(slave-1)],velocity[3*(slave-1)+1],velocity[3*(slave-1)+2]};
      row.screen.secondary_gap=secondary_gap[secondary_row];row.screen.main_gap=main_gap[m];
      row.screen.curvature=curvature[m];row.screen.margin=0x1.b504f3a13b1b3p+1;
      row.screen.drad=0x0.0p+0;row.screen.gap_load=0x0.0p+0;
      row.screen.stored_motion=0x0.0p+0;
      for(unsigned j=0;j<4;++j){const auto node=nodes[4*m+j];row.nodes[j]=node;row.constraint_codes[j]=code[node-1];
        row.screen.vertices[j]={xyz[3*(node-1)],xyz[3*(node-1)+1],xyz[3*(node-1)+2]};
        row.main_velocities[j]={velocity[3*(node-1)],velocity[3*(node-1)+1],velocity[3*(node-1)+2]};}
      result.push_back({row,2,roles[m]});
    }
  }
  {
  const int nodes[]={13,12,16,16,13,16,17,17};
  const double main_gap[]={0x1.0000000000000p-1,0x1.0000000000000p-1};
  const double curvature[]={0x0.0p+0,0x0.0p+0};
    for(unsigned m=0;m<2;++m)for(unsigned secondary_row=0;secondary_row<18;++secondary_row) {
      c::LocalRow row;const auto slave=secondary[secondary_row];bool own=false;
      for(unsigned j=0;j<4;++j)if(nodes[4*m+j]==slave)own=true;
      if(own)continue; // Native TRIVOX source exclusion precedes the packet.
      row.secondary_node=slave;row.main_count=8;row.segment_type=roles[4+m];
      row.constraint_codes[4]=code[slave-1];row.previous_dt=0.;
      row.screen.secondary={xyz[3*(slave-1)],xyz[3*(slave-1)+1],xyz[3*(slave-1)+2]};
      row.secondary_velocity={velocity[3*(slave-1)],velocity[3*(slave-1)+1],velocity[3*(slave-1)+2]};
      row.screen.secondary_gap=secondary_gap[secondary_row];row.screen.main_gap=main_gap[m];
      row.screen.curvature=curvature[m];row.screen.margin=0x1.b504f3a13b1b3p+1;
      row.screen.drad=0x0.0p+0;row.screen.gap_load=0x0.0p+0;
      row.screen.stored_motion=0x0.0p+0;
      for(unsigned j=0;j<4;++j){const auto node=nodes[4*m+j];row.nodes[j]=node;row.constraint_codes[j]=code[node-1];
        row.screen.vertices[j]={xyz[3*(node-1)],xyz[3*(node-1)+1],xyz[3*(node-1)+2]};
        row.main_velocities[j]={velocity[3*(node-1)],velocity[3*(node-1)+1],velocity[3*(node-1)+2]};}
      result.push_back({row,2,roles[m]});
    }
  }
  {
  const int nodes[]={12,15,18,18,12,18,16,16};
  const double main_gap[]={0x1.0000000000000p-1,0x1.0000000000000p-1};
  const double curvature[]={0x0.0p+0,0x0.0p+0};
    for(unsigned m=0;m<2;++m)for(unsigned secondary_row=0;secondary_row<18;++secondary_row) {
      c::LocalRow row;const auto slave=secondary[secondary_row];bool own=false;
      for(unsigned j=0;j<4;++j)if(nodes[4*m+j]==slave)own=true;
      if(own)continue; // Native TRIVOX source exclusion precedes the packet.
      row.secondary_node=slave;row.main_count=8;row.segment_type=roles[6+m];
      row.constraint_codes[4]=code[slave-1];row.previous_dt=0.;
      row.screen.secondary={xyz[3*(slave-1)],xyz[3*(slave-1)+1],xyz[3*(slave-1)+2]};
      row.secondary_velocity={velocity[3*(slave-1)],velocity[3*(slave-1)+1],velocity[3*(slave-1)+2]};
      row.screen.secondary_gap=secondary_gap[secondary_row];row.screen.main_gap=main_gap[m];
      row.screen.curvature=curvature[m];row.screen.margin=0x1.b504f3a13b1b3p+1;
      row.screen.drad=0x0.0p+0;row.screen.gap_load=0x0.0p+0;
      row.screen.stored_motion=0x0.0p+0;
      for(unsigned j=0;j<4;++j){const auto node=nodes[4*m+j];row.nodes[j]=node;row.constraint_codes[j]=code[node-1];
        row.screen.vertices[j]={xyz[3*(node-1)],xyz[3*(node-1)+1],xyz[3*(node-1)+2]};
        row.main_velocities[j]={velocity[3*(node-1)],velocity[3*(node-1)+1],velocity[3*(node-1)+2]};}
      result.push_back({row,2,roles[m]});
    }
  }
  return result;
}
} // namespace candidate_test
