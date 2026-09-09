#pragma once
// Test-only native T3 input shared by startup and prescribed-rate checks.
#include "SourcePartContactFixture.h"
#include "T3Reference.h"
#include <stdexcept>

namespace crash::qualification::source_contact {
inline tl::qualification::t3::ReferenceInput T3ReferenceInput(
    const SourcePartContactFixture& fixture,unsigned p) {
  if(!fixture.prepared() || p>=ParentCount || fixture.parents()[p].arity!=3)
    throw std::runtime_error("Native T3 reference input requires an authenticated triangle");
  tl::qualification::t3::ReferenceInput input;
  const auto& parent=fixture.parents()[p];
  for(unsigned n=0;n<3;++n) {
    const auto index=parent.local_node_indices[n]; const auto x=fixture.positions().at(index);
    input.position[n]={x.x,x.y,x.z}; input.node_ids[n]=fixture.nodes()[index].source_id;
  }
  // Exact converted source density/thickness; the named E/nu metadata does
  // not consume original MAT024 or qualify its NIP3 structural formulation.
  input.density=fixture.surface_mass().density_kg_m3;
  input.thickness=fixture.surface_mass().thickness_m;
  input.young_modulus=200e9; input.poisson_ratio=.3;
  return input;
}
}  // namespace crash::qualification::source_contact
