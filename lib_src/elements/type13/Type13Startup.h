// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss-derived RMASS, native Ileng=1 branch. See native donor inventory.
#pragma once
#include "Type13Property.h"
#include "Type13Reference.h"

namespace tl::fea::type13 {
TL_TYPE13_HD inline Status InitializeElement(const Property& property,const ReferenceInput& input,Startup& output) {
  if(!property.initialized())return Status::InvalidInput;
  Startup next;
  const auto status=InitializeReference(property.units(),input,next.reference);
  if(status!=Status::Success)return status;
  detail::UnitFactors units;
  if(!detail::ResolveUnits(property.units(),units))return Status::InvalidInput;
  // Multiply in original RMASS order before conversion to SI.
  const double mass=.5*property.mass_per_length()*next.reference.length_native;
  const double inertia=.5*property.inertia_per_length()*next.reference.length_native;
  const double added=.5*property.added_inertia_per_length()*next.reference.length_native;
  next.endpoint={mass*units.mass,inertia*units.inertia,added*units.inertia};
  if(!detail::Positive(next.endpoint.mass_kg)||!detail::Positive(next.endpoint.isotropic_inertia_kg_m2)||
     !detail::Nonnegative(next.endpoint.added_inertia_kg_m2))return Status::NonfiniteResult;
  output=next;return Status::Success;
}
} // namespace tl::fea::type13
