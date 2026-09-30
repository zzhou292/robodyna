// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Tile.h"
#include "../Layout.h"
namespace tlfea::contact::radioss_type25::runtime_detail::diagnostics {
// The owning runtime Layout places Control, flags, slots and responses in
// disjoint retained arena regions. Respond is stream-ordered after ResponseRows
// and PackForces; these independent reads cannot alias the leader's Control
// publication. No extra alias contract is imposed on a public response API.
TL_MATH_HOST_DEVICE inline Operand Read(const Device& device,std::size_t index) {
  Operand out{};
  out.positive=device.positive_flags[index]!=0;
  // Masked slots may be invalid, and an entirely inactive set needs no response
  // storage. Preserve the original flag-before-slot/response read admission.
  if(!out.positive)return out;
  const auto& response=device.responses[device.sorted_slots[index]];
  out.contact_active=response.contact_active;
  out.elastic_energy=response.normal.elastic_energy;
  out.damping_work=response.normal.damping_work;
  out.friction_work=response.friction_work;
  return out;
}
TL_MATH_HOST_DEVICE inline void Store(Tile& tile,unsigned lane,const Operand& value) {
  tile.positive[lane]=value.positive;tile.contact_active[lane]=value.contact_active;
  tile.elastic_energy[lane]=value.elastic_energy;
  tile.damping_work[lane]=value.damping_work;
  tile.friction_work[lane]=value.friction_work;
}
} // namespace tlfea::contact::radioss_type25::runtime_detail::diagnostics
