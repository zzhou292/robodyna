#pragma once

#include "PlanarWallBox.h"
#include "Q4ContactBounds.h"

namespace tlfea::contact {
enum class PrescribedNodeStatus { Ok, InvalidInput, MassFailure, FixedMotion, PenetrationLimit, NonFiniteArithmetic };
struct PrescribedNodeReport {
  PrescribedNodeStatus status=PrescribedNodeStatus::InvalidInput;
  Status mass_status=Status::kInvalidArgument;
  std::uint32_t node=UINT32_MAX,endpoint=UINT32_MAX;
  Q4IntegralInterval gaps[2]{}; // Published together only on Ok, base then endpoint.
};

// Shape-independent checks of actual free-XYZ/fully-fixed physical mass and
// one prescribed nodal segment. Neither callers nor this helper reinterpret
// component constraints as isotropic mass. No physical buffer is written.
// The parent-specific normal stencil check remains the integration caller's
// responsibility, because its shape weights are not implied by node count.
inline PrescribedNodeReport CheckPrescribedSurfaceNode(
    VectorView x0,VectorView v0,VectorView x1,VectorView v1,
    const LumpedTranslationMassView& mass,std::uint32_t node,double wall_x,double maximum_penetration) {
  PrescribedNodeReport report; report.node=node;
  if (!x0.valid() || !x1.valid() || !v0.valid() || !v1.valid() ||
      x0.node_count != mass.node_count || x1.node_count != mass.node_count ||
      v0.node_count != mass.node_count || v1.node_count != mass.node_count ||
      !IsFinite(wall_x) || !IsFinite(maximum_penetration) || maximum_penetration <= 0) return report;
  if (mass.model != TranslationMassModel::kIsotropicLumped || !mass.inverse_mass || !mass.fixed) {
    report.status=PrescribedNodeStatus::MassFailure;
    report.mass_status=mass.model != TranslationMassModel::kIsotropicLumped ?
        Status::kUnsupportedInterpolation : Status::kInvalidArgument;
    return report;
  }
  report.mass_status=mass_detail::CheckNode(mass,node);
  if (report.mass_status != Status::kOk) { report.status=PrescribedNodeStatus::MassFailure; return report; }
  const auto a=x0.at(node),b=x1.at(node),va=v0.at(node),vb=v1.at(node);
  if (!IsFinite(a) || !IsFinite(b) || !IsFinite(va) || !IsFinite(vb)) return report;
  if (mass.fixed[node] && (a.x != b.x || a.y != b.y || a.z != b.z ||
      va.x != 0 || va.y != 0 || va.z != 0 || vb.x != 0 || vb.y != 0 || vb.z != 0)) {
    report.status=PrescribedNodeStatus::FixedMotion; return report;
  }
  const double coordinate[2]={a.x,b.x};
  Q4IntegralInterval staged_gaps[2];
  for (unsigned endpoint=0;endpoint<2;++endpoint) {
    Q4IntegralInterval gap;
    if (!q4_bounds::Difference(coordinate[endpoint],wall_x,&gap)) {
      report.status=PrescribedNodeStatus::NonFiniteArithmetic; report.endpoint=endpoint; return report;
    }
    if (gap.upper > maximum_penetration) {
      report.status=PrescribedNodeStatus::PenetrationLimit; report.endpoint=endpoint; return report;
    }
    staged_gaps[endpoint]=gap;
  }
  report.gaps[0]=staged_gaps[0]; report.gaps[1]=staged_gaps[1];
  report.status=PrescribedNodeStatus::Ok; return report;
}

// Positive shape weights of Q4 or native T3 and straight nodal trajectories
// enclose the full projected sweep in these endpoint-coordinate extrema.
// This proves no intrinsic current regularity, CCD or shell-quality property.
// Native parent validation/identity belongs to the caller. Output is staged.
inline bool PrescribedSurfaceBox(VectorView base,VectorView endpoint,const std::uint32_t* nodes,
                                std::uint32_t count,double wall_x,PlanarWallBox* output) {
  if (!output || !nodes || (count != 3 && count != 4) || !base.valid() || !endpoint.valid() ||
      base.node_count != endpoint.node_count || !IsFinite(wall_x)) return false;
  PlanarWallBox next{{wall_x,DBL_MAX,DBL_MAX},{wall_x,-DBL_MAX,-DBL_MAX}};
  for (std::uint32_t n=0;n<count;++n) {
    if (nodes[n] >= base.node_count) return false;
    const Vec3 points[2]={base.at(nodes[n]),endpoint.at(nodes[n])};
    for (const auto point:points) {
      if (!IsFinite(point)) return false;
      if (point.y < next.minimum.y) next.minimum.y=point.y;
      if (point.z < next.minimum.z) next.minimum.z=point.z;
      if (point.y > next.maximum.y) next.maximum.y=point.y;
      if (point.z > next.maximum.z) next.maximum.z=point.z;
    }
  }
  *output=next; return true;
}

struct PrescribedParentInterval {
  std::uint32_t nodes[4]{},node_count=0; // Native T3 uses exactly the first 3.
  Q4IntegralInterval base_gaps[4]{},endpoint_gaps[4]{};
  PlanarWallBox physical;
};
// Shared parent preflight for native T3/Q4. No shape-specific mass stencil is
// inferred here. All result fields, including unused T3 entries, are staged.
inline PrescribedNodeReport CheckPrescribedSurfaceInterval(
    VectorView x0,VectorView v0,VectorView x1,VectorView v1,
    const LumpedTranslationMassView& mass,const std::uint32_t* nodes,std::uint32_t count,
    double wall_x,double maximum_penetration,PrescribedParentInterval* output) {
  PrescribedNodeReport report;
  if (!output || !nodes || (count != 3 && count != 4)) return report;
  PrescribedParentInterval next; next.node_count=count;
  for (std::uint32_t n=0;n<count;++n) {
    report=CheckPrescribedSurfaceNode(x0,v0,x1,v1,mass,nodes[n],wall_x,maximum_penetration);
    if (report.status != PrescribedNodeStatus::Ok) return report;
    next.nodes[n]=nodes[n]; next.base_gaps[n]=report.gaps[0]; next.endpoint_gaps[n]=report.gaps[1];
  }
  if (!PrescribedSurfaceBox(x0,x1,nodes,count,wall_x,&next.physical)) {
    report.status=PrescribedNodeStatus::InvalidInput; return report;
  }
  *output=next; return report;
}
} // namespace tlfea::contact
