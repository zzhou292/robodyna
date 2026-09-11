// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../rigid_assembly_owner/Fixture.h"
#include "lib_src/elements/type45/Model.h"

namespace type45_model_test {
namespace fe=tl::fea;
namespace joint=fe::type45;
using Fixture=rigid_assembly_owner_test::Fixture;
inline std::array<joint::JointInput,3> Inputs(const Fixture& f) {
  std::array<joint::JointInput,3> rows;
  for(unsigned kind=0;kind<3;++kind) {
    auto& row=rows[kind];
    row.property.kind=static_cast<joint::Kind>(kind+1);
    row.property.working_units=joint::WorkingUnits::MillimetreTonneSecond;
    row.property.automatic_stiffness_scale=.01;
    row.geometry.source_joint_id=2200512+kind;
    row.geometry.source_node_id[0]=10;
    row.geometry.source_node_id[1]=9302;
    if(kind) row.geometry.source_node_id[2]=9307; // Axis only, outside both rigid groups.
    for(unsigned n=0;n<(kind?3u:2u);++n)
      row.geometry.position_m[n]=f.domain.nodes()[f.domain.Find(row.geometry.source_node_id[n])].position;
    row.body[0]={fe::RigidBindingSourceKind::NodalGroup,200};
    row.body[1]={fe::RigidBindingSourceKind::Part,200}; // Same numeric ID, distinct role.
  }
  return rows;
}
} // namespace type45_model_test
