// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type45Reference.h"

namespace tl::fea::type45 {
namespace detail {
TL_TYPE45_HD inline bool InitialFrame(const GeometryInput& geometry,
                                     const Property& property, Matrix3& frame) {
  if (property.kind==Kind::Spherical) {
    frame=Columns({1,0,0},{0,1,0},{0,0,1});
    return true;
  }
  const Vec3 x=Subtract(geometry.position_m[2],geometry.position_m[0]);
  const double p1=Norm(x);
  unsigned largest=0;
  double maximum=0;
  for (unsigned i=0; i<3; ++i) {
    if (::fabs(Get(x,i))>=maximum) {
      largest=i;
      maximum=::fabs(Get(x,i));
    }
  }
  const Vec3 y=largest<2 ? Vec3{-x.y,x.x,0} : Vec3{0,x.z,-x.y};
  const Vec3 z=Cross(x,y);
  const double p2=Norm(y), p3=Norm(z);
  if (!Positive(p1) || p1<=Floors(property.working_units).axis_length ||
      !Positive(p2) || !Positive(p3)) return false;
  frame=Columns(Divide(x,p1),Divide(y,p2),Divide(z,p3));
  return Orthonormal(frame);
}
} // namespace detail

TL_TYPE45_HD inline Status Reference::Prepare(
    const Property& property, const GeometryInput& geometry,
    const DampingEndpoint (&damping)[2],
    const AutomaticStiffnessContext& context, Reference& output) {
  using namespace detail;
  if (!Valid(property)) return Status::InvalidProperty;
  const unsigned nodes=property.kind==Kind::Spherical ? 2 : 3;
  if (!geometry.source_joint_id) return Status::InvalidGeometry;
  for (unsigned i=0; i<nodes; ++i) {
    if (!geometry.source_node_id[i] || !Finite(geometry.position_m[i]))
      return Status::InvalidGeometry;
    for (unsigned j=0; j<i; ++j)
      if (geometry.source_node_id[i]==geometry.source_node_id[j]) return Status::InvalidGeometry;
  }
  if (nodes==2 && (geometry.source_node_id[2]!=0 || !Same(geometry.position_m[2],Vec3{})))
    return Status::InvalidGeometry;
  if (!Positive(context.target_dt_s)) return Status::InvalidContext;
  for (unsigned i=0; i<2; ++i) {
    const auto& main=context.main[i];
    if (main.role==EndpointRole::RigidMain) return Status::UnsupportedRole;
    if ((main.role!=EndpointRole::Structural && main.role!=EndpointRole::RigidMember) ||
        !Positive(main.mass_kg) || !Nonnegative(main.inertia_kg_m2) ||
        !Nonnegative(main.translational_stiffness_n_m) ||
        !Nonnegative(main.rotational_stiffness_nm) || !Finite(main.position_m) ||
        !Positive(damping[i].mass_kg) ||
        !Nonnegative(damping[i].mean_principal_inertia_kg_m2)) return Status::InvalidContext;
    if (main.role==EndpointRole::Structural) {
      if (damping[i].mass_kg<=Floors(property.working_units).mass20 ||
          main.source_body_id || !Same(main.position_m,geometry.position_m[i]) ||
          !Same(main.mass_kg,damping[i].mass_kg) ||
          !Same(main.inertia_kg_m2,damping[i].mean_principal_inertia_kg_m2))
        return Status::InvalidContext;
    } else if (!main.source_body_id) return Status::InvalidContext;
  }
  Reference next;
  next.property_=property;
  next.geometry_=geometry;
  next.context_=context;
  for (unsigned i=0; i<2; ++i) next.damping_[i]=damping[i];
  if (!InitialFrame(geometry,property,next.frame_)) return Status::InvalidGeometry;
  next.separation_=ToLocal(next.frame_,Subtract(geometry.position_m[1],geometry.position_m[0]));
  const double len2=Dot(next.separation_,next.separation_);
  if (!Finite(next.separation_) || !Finite(len2)) return Status::NonfiniteResult;
  next.startup_.stiffness=property.free_stiffness; // Kn=0, automatic profile.
  double kt=0, kr=0, ct=0, cr=0;
  for (unsigned i=0; i<3; ++i) {
    kt=::fmax(kt,Get(property.free_stiffness.translation,i));
    kr=::fmax(kr,Get(property.free_stiffness.rotation,i));
    ct=::fmax(ct,Get(property.free_viscosity.translation,i));
    cr=::fmax(cr,Get(property.free_viscosity.rotation,i));
  }
  next.startup_.maximum_translation_n_m=kt;
  next.startup_.maximum_rotation_nm=kr+kt*len2;
  next.startup_.maximum_viscosity_n_s_m=ct;
  next.startup_.maximum_rotational_viscosity_nm_s=cr;
  if (!Finite(next.startup_.maximum_rotation_nm) ||
      !AutomaticCoefficients(property,geometry,context,next.automatic_)) return Status::NonfiniteResult;
  next.stiffness_=property.free_stiffness;
  for (unsigned i=0; i<6; ++i) {
    if (Blocked(property.kind,i)) Set(next.stiffness_,i,i<3 ?
      next.automatic_.blocked_translation_n_m : next.automatic_.blocked_rotation_nm);
  }
  next.ready_=true;
  output=next;
  return Status::Success;
}
} // namespace tl::fea::type45
