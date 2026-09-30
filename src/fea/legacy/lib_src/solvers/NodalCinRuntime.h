// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalCinStructuralStep.h"
#include "FENodalState.h"
#include "../constraints/tied_shell/runtime/CinStageTypes.h"

namespace tl::fea {
struct NodalCinLimits {
  std::size_t max_attachments = 65536;
  std::size_t max_witnesses = 262144;
  std::size_t max_host_bytes = 128u << 20;
  std::size_t max_device_bytes = 128u << 20;
};
struct NodalCinStartup {
  // A prepared explicitly-empty model means zero attachments, but still owns
  // physical raw M/J, ordinary-node advance and the complete structural screen.
  // In that scope witness ranges/witnesses are canonical null with count zero.
  const constraints::tied_shell::TiedCinAttachmentModel* model = nullptr;
  // Complete current coefficients, before the first CIN transfer. These are
  // explicit supplied values, not inferred from reciprocal source inputs.
  // Dependent M/J may be zero; their supplied and derived conventional inverse
  // values are zero even when raw M/J is positive. Only independent free DOFs
  // require a positive raw coefficient and its reciprocal. Prepared PART members
  // keep their authenticated nonnegative M/J and zero inverse when zero.
  const double* mass = nullptr;
  const double* inertia = nullptr;
  const constraints::tied_shell::cin::WitnessRange* witness_ranges = nullptr;
  const constraints::tied_shell::cin::ActiveWitness* witnesses = nullptr;
  std::size_t witness_count = 0;
  std::uint64_t qualification_id = 0;
  NodalCinLimits limits;
};
// Read-only identity of an already admitted complete CIN roster. No physical
// coefficient or activity values are supplied or reconstructed by this view.
struct NodalCinWitnessSource {
  const constraints::tied_shell::TiedCinAttachmentModel* model = nullptr;
  const constraints::tied_shell::cin::WitnessRange* ranges = nullptr;
  const constraints::tied_shell::cin::ActiveWitness* witnesses = nullptr;
  std::size_t range_count = 0, witness_count = 0;
};
// Current scalar MS coefficients, before inversion, in SI kg. They may include
// previously accepted CIN transfer/numerical mass; they are not necessarily the
// original physical ledger. A zero dependent mass is real state. No mutable
// coefficient or standalone advancement/publication authority is exposed.
struct NodalAcceptedRawMassView {
  const double* mass_kg = nullptr;
  std::size_t node_count = 0;
  std::uint64_t owner_id = 0, base_epoch = 0, attempt = 0;
  std::uint64_t qualification_id = 0;
  cudaStream_t stream = nullptr;
};
struct NodalCinAssemblyView {
  std::uint64_t owner_id = 0, base_epoch = 0, attempt = 0, qualification_id = 0;
  double* translational_stiffness = nullptr;
  double* rotational_stiffness = nullptr;
  std::uint8_t* witness_activity = nullptr; // 0 missing, 1 active, 2 inactive.
  std::size_t node_count = 0, witness_count = 0;
  cudaStream_t stream = nullptr;
};
// The case confirms that no explicit interface stop/cleaning/release event is
// requested. Complete positive native-connectivity witnesses are checked on
// device separately. Lack of a witness rejects as Pending, never as release.
struct NodalCinAdmission {
  std::uint64_t owner_id = 0, base_epoch = 0, attempt = 0, qualification_id = 0;
  double maximum_dt = 0, maximum_rotation_increment = 0;
  bool no_explicit_interface_release_event = false;
  NodalCinStructuralStep structural; // Explicit default-off post-transfer screen.
};
struct NodalCinSnapshotBuffer {
  double* mass = nullptr;
  double* inertia = nullptr;
  double* saved_secondary_mass = nullptr;
  double* saved_secondary_inertia = nullptr;
  double* numerical_mass = nullptr;
  std::size_t capacity_nodes = 0, capacity_attachments = 0;
};
NodalReport AdvanceStaggeredCin(FENodalState&, const NodalTrialToken&, const NodalCinAdmission&);
} // namespace tl::fea
