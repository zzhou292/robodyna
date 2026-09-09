#include "Q4ParametricContact.h"

#include "Q4IntegralMeasure.h"
#include "Q4RectangularIntegration.h"

namespace tlfea::contact {
namespace {
using Code=Q4ParametricStatus;
bool SameParent(const SurfaceQ4& a,const SurfaceQ4& b) {
  if (a.feature_id != b.feature_id || a.parent_element_id != b.parent_element_id ||
      a.parent_face_id != b.parent_face_id || a.half_thickness != b.half_thickness) return false;
  for (unsigned n=0;n<4;++n) if (a.nodes[n] != b.nodes[n]) return false;
  return true;
}
bool ViewMatches(const Q4SurfaceView& view,const Q4ParametricReference& reference) {
  if (!view.positions.valid() || !view.velocities.valid() || !view.parents ||
      view.positions.node_count != reference.node_count() || view.velocities.node_count != reference.node_count() ||
      view.parent_count != reference.parent_count()) return false;
  for (unsigned p=0;p<view.parent_count;++p)
    if (!SameParent(view.parents[p],reference.parent(p).intrinsic.parent())) return false;
  return true;
}
Q4ParametricReport NodeFailure(Q4ParametricReport report) {
  report.mass_status=report.node.mass_status;
  switch (report.node.status) {
    case PrescribedNodeStatus::MassFailure:
      report.status=Code::MassFailure; report.message="Invalid or unsupported physical nodal mass"; break;
    case PrescribedNodeStatus::FixedMotion:
      report.status=Code::FixedMotion; report.message="A fully fixed node moves in the prescribed interval"; break;
    case PrescribedNodeStatus::PenetrationLimit:
      report.status=Code::IntegrationFailure; report.message="Prescribed endpoint exceeds the penetration cap";
      report.integration={Q4IntegrationStatus::PenetrationLimit,Status::kOutOfRange}; break;
    case PrescribedNodeStatus::NonFiniteArithmetic:
      report.status=Code::NonFiniteArithmetic; report.message="Unrepresentable prescribed normal gap"; break;
    default:
      report.status=Code::InvalidInput; report.message="Malformed prescribed endpoint data"; break;
  }
  return report;
}
} // namespace

Q4ParametricReport Q4ParametricReference::Initialize(VectorView positions,const SurfaceQ4* parents,
                                                     std::uint32_t count) {
  Q4ParametricReport report;
  if (prepared_ || !positions.valid() || !parents || !count || count > MaxQ4PlanarParents) return report;
  Q4ParametricReference next; next.node_count_=positions.node_count; next.parent_count_=count;
  for (std::uint32_t p=0;p<count;++p) {
    report.parent=p;
    for (std::uint32_t previous=0;previous<p;++previous)
      if (parents[p].feature_id == parents[previous].feature_id ||
          parents[p].parent_element_id == parents[previous].parent_element_id) {
        report.message="Duplicate Q4 feature or parent identity"; return report;
      }
    auto& saved=next.parents_[p];
    report.reference_status=PrepareQ4MaterialMeasure(positions,parents[p],&saved.intrinsic);
    if (report.reference_status != SurfaceMeasureStatus::Ok) {
      report.status=Code::ReferenceFailure; report.message="Intrinsic reference Q4 could not be certified"; return report;
    }
    Q4CertifiedIntegral density;
    report.reference_status=EvaluateQ4MaterialDensity(saved.intrinsic,0,0,&density);
    Q4IntegralInterval area;
    const double value=4*density.value;
    if (report.reference_status != SurfaceMeasureStatus::Ok || !IsFinite(value) || !(value > 0) ||
        !q4_bounds::Scale({density.lower,density.upper},4,&area) || !(area.lower > 0) ||
        value < area.lower || value > area.upper || !q4_bounds::Certify(value,area,&saved.area)) {
      report.status=Code::NonFiniteArithmetic;
      report.message="Center-area measure cannot meet the immutable area-certificate contract"; return report;
    }
  }
  next.prepared_=true; *this=next;
  report.status=Code::Ok; report.message="Uniform-natural center-area Q4 reference prepared";
  report.parent=UINT32_MAX; return report;
}

Q4ParametricReport IntegrateQ4ParametricContact(
    const PlanarWallGeometry& wall,const Q4ParametricReference& reference,
    const Q4SurfaceView& base,const Q4SurfaceView& endpoint,const LumpedTranslationMassView& mass,
    const Q4ParametricConfig& config,std::uint64_t attempt,Q4RectangularScratch scratch,Q4ParametricResult* output) {
  Q4ParametricReport report;
  if (!output || !attempt || !reference.prepared() ||
      config.measure != Q4ReferenceContactMeasure::CenterAreaUniformNatural ||
      !IsFinite(config.stiffness_per_area) || config.stiffness_per_area <= 0 ||
      !IsFinite(config.maximum_penetration) || config.maximum_penetration <= 0 ||
      !ViewMatches(base,reference) || !ViewMatches(endpoint,reference)) return report;
  if (!wall.initialized()) {
    report.status=Code::GeometryFailure; report.message="Finite wall has not been prepared";
    report.geometry={PlanarContactStatus::NotInitialized,report.message}; return report;
  }
  if (mass.node_count != reference.node_count()) {
    report.status=Code::MassFailure; report.message="Physical mass extent differs from reference node space"; return report;
  }
  Q4ParametricResult next; next.parent_count=reference.parent_count(); next.measure=config.measure;
  // Complete every parent preflight before touching the caller's integration
  // scratch. Shared physical nodes are checked consistently, never duplicated
  // into independent dynamics nodes or merged into invented shape weights.
  for (std::uint32_t p=0;p<reference.parent_count();++p) {
    report.parent=p;
    const auto& parent=reference.parent(p).intrinsic.parent();
    NormalJacobian center;
    report.mass_status=BuildQ4NormalXJacobian(mass,parent,0,0,attempt,&center);
    if (report.mass_status != Status::kOk) {
      report.status=Code::MassFailure; report.message="Invalid physical Q4 center normal stencil"; return report;
    }
    PrescribedParentInterval interval;
    report.node=CheckPrescribedSurfaceInterval(base.positions,base.velocities,endpoint.positions,endpoint.velocities,
                                              mass,parent.nodes,4,wall.wall_x(),config.maximum_penetration,&interval);
    if (report.node.status != PrescribedNodeStatus::Ok) return NodeFailure(report);
    report.geometry=CheckPlanarWallBox(wall,interval.physical,config.exposed_clearance,parent.feature_id,
                                      PlanarWallBoxMode::ConservativeExpansion,&next.coverage[p]);
    if (report.geometry.status != PlanarContactStatus::Ok) {
      report.status=Code::GeometryFailure; report.message=report.geometry.message;
      report.geometry.sample=p; return report;
    }
  }
  for (std::uint32_t p=0;p<reference.parent_count();++p) {
    report.parent=p; const auto& area=reference.parent(p).area;
    const Q4PrescribedNormalIntegrationInput input{endpoint,mass,p,attempt,wall.wall_x(),area.value,
                                                  config.stiffness_per_area,config.maximum_penetration};
    report.integration=IntegrateQ4NormalContactRectangular(input,config.integration,scratch,&next.parents[p]);
    if (report.integration.status != Q4IntegrationStatus::Ok) {
      report.status=Code::IntegrationFailure; report.message="Prescribed parametric Q4 integral failed"; return report;
    }
    auto& integral=next.parents[p].integration;
    if (!ExpandQ4IntegralMeasure(area.value,{area.lower,area.upper},&integral)) {
      report.status=Code::NonFiniteArithmetic; report.message="Immutable center-area expansion failed"; return report;
    }
    if (!WithinQ4IntegralBudgets(integral,config.integration)) {
      report.status=Code::IntegrationFailure; report.message="Center-area certificate exceeds unchanged integral budgets";
      report.integration={Q4IntegrationStatus::UnattainableAccuracy,Status::kOutOfRange}; return report;
    }
  }
  next.valid=true; *output=next;
  report.status=Code::Ok; report.message="Prescribed uniform-natural center-area Q4 contact accepted";
  report.parent=UINT32_MAX; return report;
}
} // namespace tlfea::contact
