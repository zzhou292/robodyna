#include "PrescribedSurfaceContact.h"

#include "Q4IntegralMeasure.h"
#include "Q4RectangularIntegration.h"
#include "T3ContactIntegration.h"

namespace tlfea::contact {
namespace {
using Code=PrescribedSurfaceStatus;
using Family=PrescribedSurfaceFamily;
bool Empty(const PrescribedQ4Request& request) {
  return !request.reference && !request.reference_parent && !request.base && !request.endpoint;
}
bool Empty(const PrescribedT3Request& request) { return !request.reference && !request.base && !request.endpoint; }
bool MatchingViews(const PrescribedSurfaceInput& input) {
  return input.base_positions.valid() && input.base_velocities.valid() && input.endpoint_positions.valid() &&
      input.endpoint_velocities.valid() && input.base_velocities.node_count==input.base_positions.node_count &&
      input.endpoint_positions.node_count==input.base_positions.node_count &&
      input.endpoint_velocities.node_count==input.base_positions.node_count;
}
struct NativeBinding {
  const std::uint32_t* nodes=nullptr;
  std::uint32_t count=0;
  std::uint64_t feature=0,parent=0;
};
PrescribedSurfaceReport CheckBinding(const PrescribedSurfaceParent& request,std::uint32_t count,
                                    NativeBinding* output) {
  PrescribedSurfaceReport report; NativeBinding next;
  if (request.family==Family::Q4CenterAreaUniformNatural) {
    const auto& q=request.q4;
    if (!Empty(request.t3) || !q.reference || !q.base || !q.endpoint) return report;
    if (!q.reference->prepared() || q.reference_parent>=q.reference->parent_count() || q.reference->node_count()!=count) {
      report.status=Code::ReferenceFailure; report.message="Invalid Q4 reference selector or prepared global node extent"; return report;
    }
    const auto& parent=q.reference->parent(q.reference_parent).intrinsic.parent();
    if (!q4_detail::SameParent(parent,*q.base) || !q4_detail::SameParent(parent,*q.endpoint)) return report;
    next={parent.nodes,4,parent.feature_id,parent.parent_element_id};
  } else if (request.family==Family::T3NativeLinear) {
    const auto& t=request.t3;
    if (!Empty(request.q4) || !t.reference || !t.base || !t.endpoint) return report;
    if (!t.reference->prepared()) {
      report.status=Code::ReferenceFailure; report.message="Native triangle reference is not prepared"; return report;
    }
    const auto& parent=t.reference->parent();
    if (!t3_integration::SameParent(parent,*t.base) || !t3_integration::SameParent(parent,*t.endpoint)) return report;
    next={parent.nodes,3,parent.feature_id,parent.parent_element_id};
  } else return report;
  *output=next; report.status=Code::Ok; return report;
}
void NodeFailure(PrescribedSurfaceReport* report,Family family) {
  report->mass_status=report->node.mass_status;
  switch (report->node.status) {
    case PrescribedNodeStatus::MassFailure:
      report->status=Code::MassFailure; report->message="Invalid or unsupported physical nodal mass"; break;
    case PrescribedNodeStatus::FixedMotion:
      report->status=Code::FixedMotion; report->message="A fully fixed node moves in the prescribed interval"; break;
    case PrescribedNodeStatus::PenetrationLimit:
      report->status=Code::IntegrationFailure; report->message="Prescribed endpoint exceeds the penetration cap";
      if (family==Family::Q4CenterAreaUniformNatural) report->q4={Q4IntegrationStatus::PenetrationLimit,Status::kOutOfRange};
      else report->t3={T3IntegrationStatus::PenetrationLimit,Status::kOutOfRange};
      break;
    case PrescribedNodeStatus::NonFiniteArithmetic:
      report->status=Code::NonFiniteArithmetic; report->message="Unrepresentable prescribed normal gap"; break;
    default:
      report->status=Code::InvalidInput; report->message="Malformed prescribed endpoint data"; break;
  }
}
} // namespace

PrescribedSurfaceReport IntegratePrescribedSurfaceContact(
    const PlanarWallGeometry& wall,const PrescribedSurfaceInput& input,const PrescribedSurfaceConfig& config,
    std::uint64_t attempt,Q4RectangularScratch scratch,PrescribedSurfaceResult* output) {
  PrescribedSurfaceReport report;
  if (!output || !attempt || !input.parents || !input.parent_count || input.parent_count>MaxQ4PlanarParents ||
      !MatchingViews(input) || !IsFinite(config.stiffness_per_area) || config.stiffness_per_area<=0 ||
      !IsFinite(config.maximum_penetration) || config.maximum_penetration<=0) return report;
  if (!wall.initialized()) {
    report.status=Code::GeometryFailure; report.message="Finite wall has not been prepared";
    report.geometry={PlanarContactStatus::NotInitialized,report.message}; return report;
  }
  if (input.mass.node_count!=input.base_positions.node_count) {
    report.status=Code::MassFailure; report.message="Physical mass extent differs from prescribed node space"; return report;
  }
  PrescribedSurfaceResult next; next.parent_count=input.parent_count;
  NativeBinding binding[MaxQ4PlanarParents]{};
  for (std::uint32_t p=0;p<input.parent_count;++p) {
    const auto& request=input.parents[p]; const auto family=request.family;
    report=CheckBinding(request,input.base_positions.node_count,&binding[p]); report.parent=p;
    if (report.status!=Code::Ok) return report;
    for (std::uint32_t previous=0;previous<p;++previous)
      if (binding[p].feature==binding[previous].feature || binding[p].parent==binding[previous].parent) {
        report.status=Code::InvalidInput; report.message="Duplicate prescribed source feature or parent identity"; return report;
      }
    NormalJacobian center;
    if (family==Family::Q4CenterAreaUniformNatural)
      report.mass_status=BuildQ4NormalXJacobian(input.mass,*request.q4.endpoint,0,0,attempt,&center);
    else {
      const double weights[3]={1./3,1./3,1./3};
      report.mass_status=BuildLinearTriangleNormalJacobian(input.mass,*request.t3.endpoint,weights,nullptr,nullptr,
                                                         {-1,0,0},attempt,&center);
    }
    if (report.mass_status!=Status::kOk) {
      report.status=Code::MassFailure; report.message="Invalid physical native-parent center normal stencil"; return report;
    }
    PrescribedParentInterval interval;
    report.node=CheckPrescribedSurfaceInterval(input.base_positions,input.base_velocities,input.endpoint_positions,
        input.endpoint_velocities,input.mass,binding[p].nodes,binding[p].count,wall.wall_x(),config.maximum_penetration,&interval);
    if (report.node.status!=PrescribedNodeStatus::Ok) { NodeFailure(&report,family); return report; }
    report.geometry=CheckPlanarWallBox(wall,interval.physical,config.exposed_clearance,binding[p].feature,
                                      PlanarWallBoxMode::ConservativeExpansion,&next.parents[p].coverage);
    if (report.geometry.status!=PlanarContactStatus::Ok) {
      report.status=Code::GeometryFailure; report.message=report.geometry.message; report.geometry.sample=p; return report;
    }
    next.parents[p].family=family;
  }
  // Keep the legacy Q4 geometry/mass rejection priority. Family limits still
  // all pass before the first raw integral can touch scratch or local results.
  for (std::uint32_t p=0;p<input.parent_count;++p) {
    const auto family=input.parents[p].family; report.parent=p;
    if ((family==Family::Q4CenterAreaUniformNatural && !q4_rectangular::ValidResources(config.q4,scratch)) ||
        (family==Family::T3NativeLinear && !t3_integration::ValidLimits(config.t3))) {
      report.status=Code::IntegrationFailure; report.message="Invalid selected-family integral limits or scratch";
      if (family==Family::Q4CenterAreaUniformNatural) report.q4={Q4IntegrationStatus::InvalidInput,Status::kInvalidArgument};
      else report.t3={T3IntegrationStatus::InvalidInput,Status::kInvalidArgument};
      return report;
    }
  }
  for (std::uint32_t p=0;p<input.parent_count;++p) {
    const auto& request=input.parents[p]; report.parent=p;
    if (request.family==Family::Q4CenterAreaUniformNatural) {
      const auto& area=request.q4.reference->parent(request.q4.reference_parent).area;
      const Q4SurfaceView endpoint{input.endpoint_positions,input.endpoint_velocities,request.q4.endpoint,1};
      const Q4PrescribedNormalIntegrationInput raw{endpoint,input.mass,0,attempt,wall.wall_x(),area.value,
                                                  config.stiffness_per_area,config.maximum_penetration};
      report.q4=IntegrateQ4NormalContactRectangular(raw,config.q4,scratch,&next.parents[p].q4);
      if (report.q4.status!=Q4IntegrationStatus::Ok) {
        report.status=Code::IntegrationFailure; report.message="Prescribed parametric Q4 integral failed"; return report;
      }
      auto& integral=next.parents[p].q4.integration;
      if (!ExpandQ4IntegralMeasure(area.value,{area.lower,area.upper},&integral)) {
        report.status=Code::NonFiniteArithmetic; report.message="Immutable center-area expansion failed"; return report;
      }
      if (!WithinQ4IntegralBudgets(integral,config.q4)) {
        report.status=Code::IntegrationFailure; report.message="Center-area certificate exceeds unchanged integral budgets";
        report.q4={Q4IntegrationStatus::UnattainableAccuracy,Status::kOutOfRange}; return report;
      }
    } else {
      const LinearTriangleSurfaceView endpoint{input.endpoint_positions,input.endpoint_velocities,
                                              input.mass.inverse_mass,request.t3.endpoint,1};
      const T3NormalIntegrationInput raw{request.t3.reference,endpoint,input.mass,0,attempt,wall.wall_x(),
                                        config.stiffness_per_area,config.maximum_penetration};
      report.t3=IntegrateT3NormalContact(raw,config.t3,&next.parents[p].t3);
      if (report.t3.status!=T3IntegrationStatus::Ok) {
        report.status=Code::IntegrationFailure; report.message="Prescribed native triangle integral failed"; return report;
      }
    }
  }
  next.valid=true; *output=next;
  report.status=Code::Ok; report.message="Prescribed native surface contact accepted"; report.parent=UINT32_MAX; return report;
}
} // namespace tlfea::contact
