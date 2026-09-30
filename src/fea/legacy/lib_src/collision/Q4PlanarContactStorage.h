#pragma once

#include "Q4PlanarContact.h"
#include "Q4PlanarStiffness.h"
#include "PlanarWallGeometry.h"
#include "Q4RectangularIntegrationTypes.h"

#include <array>
#include <type_traits>

namespace tlfea::contact::q4_planar_detail {
constexpr std::size_t MaxIncidentNodes = MaxQ4PlanarNodes;
struct Model {
  Q4PlanarContactConfig config;
  PreparedQ4PlanarParent reference[MaxQ4PlanarParents]{};
  SurfaceQ4 parents[MaxQ4PlanarParents]{};
  std::uint32_t parent_count = 0;
  double wall_x = 0, wall_tolerance = 0;
  double inverse_mass[tl::fea::MaxTranslationNodes]{};
  std::uint8_t fixed[tl::fea::MaxTranslationNodes]{};
  Q4PlanarStiffness stiffness;
};
struct Control {
  Q4PlanarContactDiagnostics diagnostics;
  Q4PlanarContactStatus status = Q4PlanarContactStatus::Ok;
  std::uint32_t parent = UINT32_MAX, node = UINT32_MAX;
  PlanarContactStatus geometry = PlanarContactStatus::Ok;
  Q4IntegrationReport integration;
};
struct Base {
  Q4PlanarContactDiagnostics diagnostics;
  double force[MaxIncidentNodes]{}, force_error[MaxIncidentNodes]{};
  double addition_error[MaxIncidentNodes]{};
};
struct Storage {
  Model model;
  Control control;
  Base base;
  Q4PlanarParentResult result[MaxQ4PlanarParents]{};
  double force[MaxIncidentNodes]{}, force_error[MaxIncidentNodes]{};
  Q4IntegralInterval force_truth[MaxIncidentNodes]{};
  double staged_total[MaxIncidentNodes]{}, addition_error[MaxIncidentNodes]{};
  std::uint32_t heap[MaxQ4IntegrationLeaves]{};
  // Points into this allocation's selected leaf-array tail. Initialization
  // explicitly constructs the selected array; its type never changes.
  void* leaves = nullptr;
};
constexpr bool ValidBackend(Q4PlanarIntegrationBackend backend) {
  return backend == Q4PlanarIntegrationBackend::ScalarDyadicSquares ||
         backend == Q4PlanarIntegrationBackend::RectangularDyadic;
}
constexpr std::size_t DeviceBytes(Q4PlanarIntegrationBackend backend) {
  return backend == Q4PlanarIntegrationBackend::ScalarDyadicSquares ?
      sizeof(Storage)+MaxQ4IntegrationLeaves*sizeof(Q4IntegrationCell) :
      backend == Q4PlanarIntegrationBackend::RectangularDyadic ?
      sizeof(Storage)+MaxQ4IntegrationLeaves*sizeof(Q4RectangularCell) : 0;
}
static_assert(alignof(Storage) >= alignof(Q4IntegrationCell) && alignof(Storage) >= alignof(Q4RectangularCell) &&
              sizeof(Storage)%alignof(Q4IntegrationCell) == 0 && sizeof(Storage)%alignof(Q4RectangularCell) == 0,
              "The selected tail must retain its cell alignment");
static_assert(std::is_trivially_copyable_v<Q4IntegrationCell> && std::is_trivially_destructible_v<Q4IntegrationCell> &&
              std::is_trivially_copyable_v<Q4RectangularCell> && std::is_trivially_destructible_v<Q4RectangularCell>,
              "Selected scratch lifetimes must end safely with the owning allocation");
static_assert(DeviceBytes(Q4PlanarIntegrationBackend::ScalarDyadicSquares) <= MaxPlanarContactDeviceBytes &&
              DeviceBytes(Q4PlanarIntegrationBackend::RectangularDyadic) <= MaxPlanarContactDeviceBytes,
              "Either complete selected Q4 storage must fit the declared 1 MiB cap");
}  // namespace tlfea::contact::q4_planar_detail

namespace tlfea::contact {
struct Q4PlanarContact::Impl {
  ~Impl();
  q4_planar_detail::Storage* device = nullptr;
  std::size_t device_bytes = 0;
  Q4PlanarContactConfig config;
  PlanarWallGeometry wall;
  q4_planar_detail::Control control;
  std::array<Q4PlanarParentResult,MaxQ4PlanarParents> staging;
  std::uint32_t parent_count = 0;
  double rate_bound = 0;
  std::uint64_t last_epoch = 0, last_attempt = 0;
  cudaStream_t last_stream = nullptr;
  bool usable = true, has_base = false, has_results = false;
  Q4PlanarContactReport Check(cudaError_t);
  Q4PlanarContactReport ConstructLeaves();
  Q4PlanarContactReport ReadControl(cudaStream_t);
  Q4PlanarContactReport FailAssembly(const tl::fea::NodalAssemblyView&,Q4PlanarContactReport);
};
}  // namespace tlfea::contact
