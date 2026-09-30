// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type45AutomaticStiffness.h"

namespace tl::fea::type45 {
class Reference {
 public:
  TL_TYPE45_HD bool ready() const { return ready_; }
  TL_TYPE45_HD const Property& property() const { return property_; }
  TL_TYPE45_HD const GeometryInput& geometry() const { return geometry_; }
  TL_TYPE45_HD const Matrix3& frame() const { return frame_; }
  TL_TYPE45_HD Vec3 local_separation_m() const { return separation_; }
  TL_TYPE45_HD const DampingEndpoint& damping(unsigned i) const { return damping_[i]; }
  TL_TYPE45_HD const AutomaticStiffnessContext& context() const { return context_; }
  TL_TYPE45_HD const AutomaticStiffness& automatic_stiffness() const { return automatic_; }
  TL_TYPE45_HD const DofValues& stiffness() const { return stiffness_; }
  TL_TYPE45_HD const StartupValues& startup() const { return startup_; }

  TL_TYPE45_HD static Status Prepare(
      const Property& property, const GeometryInput& geometry,
      const DampingEndpoint (&damping)[2],
      const AutomaticStiffnessContext& context, Reference& output);
  TL_TYPE45_HD bool Matches(const Reference& other) const;

 private:
  bool ready_=false;
  Property property_{};
  GeometryInput geometry_{};
  Matrix3 frame_{};
  Vec3 separation_{};
  DampingEndpoint damping_[2]{};
  AutomaticStiffnessContext context_{};
  AutomaticStiffness automatic_{};
  DofValues stiffness_{};
  StartupValues startup_{};
};
} // namespace tl::fea::type45

#include "Type45ReferencePrepare.h"
#include "Type45ReferenceIdentity.h"
