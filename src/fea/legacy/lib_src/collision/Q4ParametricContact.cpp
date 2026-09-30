#include "Q4ParametricContact.h"

#include "PrescribedSurfaceContact.h"

namespace tlfea::contact {
namespace {
using Code=Q4ParametricStatus;
bool ViewMatches(const Q4SurfaceView& view,const Q4ParametricReference& reference) {
  if (!view.positions.valid() || !view.velocities.valid() || !view.parents ||
      view.positions.node_count != reference.node_count() || view.velocities.node_count != reference.node_count() ||
      view.parent_count != reference.parent_count()) return false;
  for (unsigned p=0;p<view.parent_count;++p)
    if (!q4_detail::SameParent(view.parents[p],reference.parent(p).intrinsic.parent())) return false;
  return true;
}
Code LegacyStatus(PrescribedSurfaceStatus status) {
  switch (status) {
    case PrescribedSurfaceStatus::Ok: return Code::Ok;
    case PrescribedSurfaceStatus::InvalidInput: return Code::InvalidInput;
    case PrescribedSurfaceStatus::ReferenceFailure: return Code::ReferenceFailure;
    case PrescribedSurfaceStatus::GeometryFailure: return Code::GeometryFailure;
    case PrescribedSurfaceStatus::MassFailure: return Code::MassFailure;
    case PrescribedSurfaceStatus::FixedMotion: return Code::FixedMotion;
    case PrescribedSurfaceStatus::IntegrationFailure: return Code::IntegrationFailure;
    case PrescribedSurfaceStatus::NonFiniteArithmetic: return Code::NonFiniteArithmetic;
  }
  return Code::InvalidInput;
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
  PrescribedSurfaceParent parents[MaxQ4PlanarParents];
  for (std::uint32_t p=0;p<reference.parent_count();++p) {
    parents[p].family=PrescribedSurfaceFamily::Q4CenterAreaUniformNatural;
    parents[p].q4={&reference,p,&base.parents[p],&endpoint.parents[p]};
  }
  const PrescribedSurfaceInput input{base.positions,base.velocities,endpoint.positions,endpoint.velocities,
                                     mass,parents,reference.parent_count()};
  const PrescribedSurfaceConfig selected{config.stiffness_per_area,config.maximum_penetration,
                                         config.exposed_clearance,config.integration,{}};
  PrescribedSurfaceResult result;
  const auto common=IntegratePrescribedSurfaceContact(wall,input,selected,attempt,scratch,&result);
  report.status=LegacyStatus(common.status); report.message=common.message; report.parent=common.parent;
  report.geometry=common.geometry; report.node=common.node; report.mass_status=common.mass_status; report.integration=common.q4;
  if (report.status!=Code::Ok) return report;
  Q4ParametricResult next; next.parent_count=result.parent_count; next.measure=config.measure;
  for (std::uint32_t p=0;p<result.parent_count;++p) {
    next.coverage[p]=result.parents[p].coverage; next.parents[p]=result.parents[p].q4;
  }
  next.valid=true; *output=next;
  report.status=Code::Ok; report.message="Prescribed uniform-natural center-area Q4 contact accepted";
  report.parent=UINT32_MAX; return report;
}
} // namespace tlfea::contact
