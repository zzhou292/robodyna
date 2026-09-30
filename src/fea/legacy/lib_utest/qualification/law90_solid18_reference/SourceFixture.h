// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
#include "RadiatorSourceFixture.h"
namespace law90_reference_test {
namespace fixture = radiator_source_fixture;
inline s::ReferenceInput Original(unsigned row,bool working=false) {
  s::ReferenceInput input;
  input.profile=t::Law90Profile();
  input.source_element_id=fixture::solid_records_u64[row][0];
  input.source_part_id=fixture::solid_records_u64[row][1];
  input.source_section_id=fixture::section_id;
  input.source_material_id=fixture::material_id;
  input.density_kg_m3=working ? fixture::density_working : fixture::density_si;
  for(unsigned n=0;n<8;++n) {
    const auto local=fixture::solid_nodes_local_u32[row][n];
    input.source_node_id[n]=fixture::node_ids_u64[local][0];
    const auto& x=working ? fixture::node_position_mm_f64[local] : fixture::node_position_m_f64[local];
    input.position_m[n]={x[0],x[1],x[2]};
    assert(input.source_node_id[n]==fixture::solid_records_u64[row][n+2]);
  }
  return input;
}
inline t::KinematicsInput OriginalCurrent(const s::ReferenceInput& input) {
  auto result=Current(input);
  result.dt_s=1e-6;
  const auto origin=input.position_m[0];
  for(unsigned n=0;n<8;++n) {
    const auto p=input.position_m[n];
    const s::Vec3 d{p.x-origin.x,p.y-origin.y,p.z-origin.z};
    const s::Vec3 v{20*d.x+7*d.y,-11*d.y+5*d.z,4*d.z-9*d.x};
    result.velocity_m_s[n]=v;
    result.position_m[n]={p.x+result.dt_s*v.x,p.y+result.dt_s*v.y,p.z+result.dt_s*v.z};
  }
  return result;
}
}
