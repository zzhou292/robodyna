#pragma once

#include "Q4PlanarContact.h"
#include "Q4RectangularIntegration.h"

namespace tlfea::contact::q4_planar_detail {

// Execution adapter only. Both operations use the same immutable physical
// input, absolute limits and staged C4 parent result. All C3 area expansion,
// assembly and interval-work arithmetic remains in the common diagnostics.
TL_SURFACE_HD inline Q4IntegrationReport IntegrateParent(const Q4NormalIntegrationInput& input,
    const Q4PlanarContactConfig& config,void* leaves,std::uint32_t* heap,Q4PlanarParentResult* parent) {
  Q4IntegrationReport report;
  switch (config.integration_backend) {
    case Q4PlanarIntegrationBackend::ScalarDyadicSquares:
      report=IntegrateQ4NormalContact(input,config.integration,
          {static_cast<Q4IntegrationCell*>(leaves),heap,MaxQ4IntegrationLeaves,MaxQ4IntegrationLeaves},&parent->integration);
      if (report.status == Q4IntegrationStatus::Ok) {
        parent->integration_backend=config.integration_backend;
        parent->deepest_u=parent->integration.deepest_leaf;
        parent->deepest_v=parent->integration.deepest_leaf;
      }
      return report;
    case Q4PlanarIntegrationBackend::RectangularDyadic: {
      Q4RectangularResult result;
      report=IntegrateQ4NormalContactRectangular(input,config.integration,
          {static_cast<Q4RectangularCell*>(leaves),heap,MaxQ4IntegrationLeaves,MaxQ4IntegrationLeaves},&result);
      if (report.status == Q4IntegrationStatus::Ok) {
        parent->integration=result.integration;
        parent->integration_backend=config.integration_backend;
        parent->deepest_u=result.deepest_u; parent->deepest_v=result.deepest_v;
      }
      return report;
    }
  }
  return {Q4IntegrationStatus::InvalidInput,Status::kInvalidArgument};
}
}  // namespace tlfea::contact::q4_planar_detail
