#include "Q4PrescribedPlanarContact.h"

#include "Q4IntegralMeasure.h"
#include "Q4RectangularIntegration.h"

namespace tlfea::contact {
namespace {
using Code=Q4PrescribedPlanarStatus;

bool SamePosition(Vec3 a,Vec3 b) { return a.x == b.x && a.y == b.y && a.z == b.z; }
bool Zero(Vec3 a) { return a.x == 0 && a.y == 0 && a.z == 0; }
bool WithinLimits(const Q4IntegrationResult& result,const Q4IntegrationLimits& limits) {
  if (result.resultant.error > limits.force_error || result.potential.error > limits.energy_error) return false;
  for (const auto& force:result.force) if (force.error > limits.force_error) return false;
  return true;
}
Q4PrescribedPlanarReport CheckMassAndEndpoints(
    Q4PrescribedPlanarReport report,Q4PlanarReferenceView reference,
    const Q4SurfaceView& base,const Q4SurfaceView& endpoint,const LumpedTranslationMassView& mass,
    double maximum_penetration,std::uint64_t attempt) {
  if (mass.node_count != reference.global_node_count) {
    report.status=Code::MassFailure; report.message="Physical mass extent differs from the Q4 node space";
    report.mass_status=Status::kInvalidArgument; return report;
  }
  for (std::uint32_t p=0;p<reference.parent_count;++p) {
    const auto& parent=reference.parents[p].parent;
    NormalJacobian check;
    report.mass_status=BuildQ4NormalXJacobian(mass,parent,0,0,attempt,&check);
    if (report.mass_status != Status::kOk) {
      report.status=Code::MassFailure; report.message="Invalid or unsupported physical Q4 normal mass";
      report.parent=p; return report;
    }
    for (const auto node:parent.nodes) {
      const auto x0=base.positions.at(node),x1=endpoint.positions.at(node);
      if (mass.fixed[node] && (!SamePosition(x0,x1) || !Zero(base.velocities.at(node)) ||
                               !Zero(endpoint.velocities.at(node)))) {
        report.status=Code::FixedMotion; report.message="A fully fixed physical node moves in the prescribed interval";
        report.parent=p; return report;
      }
      const double coordinate[2]={x0.x,x1.x};
      for (unsigned end=0;end<2;++end) {
        Q4IntegralInterval gap;
        if (!q4_bounds::Difference(coordinate[end],reference.wall_x,&gap)) {
          report.status=Code::NonFiniteArithmetic; report.message="Unrepresentable endpoint normal gap";
          report.parent=p; return report;
        }
        if (gap.upper > maximum_penetration) {
          report.status=Code::IntegrationFailure;
          report.message=end == 0 ? "Base endpoint exceeds the declared penetration cap" :
                                   "Candidate endpoint exceeds the declared penetration cap";
          report.integration={Q4IntegrationStatus::PenetrationLimit,Status::kOutOfRange};
          report.parent=p; return report;
        }
      }
    }
  }
  report.status=Code::Ok; return report;
}
}  // namespace

Q4PrescribedPlanarReport IntegrateQ4PrescribedPlanarContact(
    const PlanarWallGeometry& wall,Q4PlanarReferenceView reference,
    const Q4SurfaceView& base,const Q4SurfaceView& endpoint,
    const LumpedTranslationMassView& mass,const Q4PrescribedPlanarConfig& config,
    std::uint64_t attempt,Q4RectangularScratch scratch,Q4PrescribedPlanarResult* output) {
  Q4PrescribedPlanarReport report;
  if (!output || !attempt || !IsFinite(config.stiffness_per_area) || config.stiffness_per_area <= 0 ||
      !IsFinite(config.maximum_penetration) || config.maximum_penetration <= 0) return report;
  Q4PrescribedPlanarResult staged;
  report.geometry=CheckQ4PlanarSweep(wall,reference,base,endpoint,config.sweep,&staged.geometry);
  if (report.geometry.status != PlanarContactStatus::Ok) {
    report.status=Code::GeometryFailure; report.message=report.geometry.message;
    report.parent=report.geometry.sample; return report;
  }
  report=CheckMassAndEndpoints(report,reference,base,endpoint,mass,config.maximum_penetration,attempt);
  if (report.status != Code::Ok) return report;
  for (std::uint32_t p=0;p<reference.parent_count;++p) {
    const auto& saved=reference.parents[p];
    const Q4PrescribedNormalIntegrationInput input{endpoint,mass,p,attempt,reference.wall_x,
        saved.projected_area,config.stiffness_per_area,config.maximum_penetration};
    report.integration=IntegrateQ4NormalContactRectangular(input,config.integration,scratch,&staged.parents[p]);
    if (report.integration.status != Q4IntegrationStatus::Ok) {
      report.status=Code::IntegrationFailure; report.message="Prescribed endpoint contact integral failed";
      report.parent=p; return report;
    }
    auto& integral=staged.parents[p].integration;
    if (!ExpandQ4IntegralMeasure(saved.projected_area,saved.area_enclosure,&integral)) {
      report.status=Code::NonFiniteArithmetic; report.message="Exact reference measure expansion failed";
      report.parent=p; return report;
    }
    if (!WithinLimits(integral,config.integration)) {
      report.status=Code::IntegrationFailure; report.message="Expanded reference measure exceeds declared integral budgets";
      report.integration.status=Q4IntegrationStatus::UnattainableAccuracy;
      report.integration.cause=Status::kOutOfRange; report.parent=p; return report;
    }
  }
  staged.valid=true; *output=staged;
  report.status=Code::Ok; report.message="Prescribed Q4 sweep and endpoint contact integrals accepted";
  report.parent=UINT32_MAX; return report;
}
}  // namespace tlfea::contact
