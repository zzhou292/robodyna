// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "qeph/QephBatch.h"
#include "t3/T3Batch.h"
#include "qbat/QbatBatch.h"
#include "type25/Type25Batch.h"
#include "type13/resident/Batch.h"
#include "solids/resident/Batch.h"

namespace tl::fea {
namespace qeph { class QephBatch; }
namespace t3 { class T3Batch; }
namespace qbat { class Batch; }
namespace type25 { class Batch; }

// Every nonempty producer in the physical ledger has exactly one participant.
// Point masses have no constitutive cache and require no phantom participant.
struct ShellPhysicalParticipants {
  qeph::QephBatch* qeph = nullptr;
  t3::T3Batch* t3 = nullptr;
  qbat::Batch* qbat = nullptr;
  type25::Batch* type25 = nullptr;
  type13::Batch* type13 = nullptr;
  solids::Batch* solids = nullptr;
};
struct ShellPhysicalCandidates {
  const qeph::BatchDiagnostics* qeph = nullptr;
  const t3::BatchDiagnostics* t3 = nullptr;
  const qbat::BatchDiagnostics* qbat = nullptr;
  const type25::BatchDiagnostics* type25 = nullptr;
  const type13::BatchDiagnostics* type13 = nullptr;
  const solids::BatchDiagnostics* solids = nullptr;
};
// Typed participant diagnostics are retained without recomputing their signed
// work. Kinetic energy is explicitly unavailable: immutable startup coefficients
// cannot measure the current CIN / rigid-primary kinetic state.
struct ShellPhysicalDiagnostics {
  NodalStamp base_stamp;
  qeph::BatchDiagnostics qeph;
  t3::BatchDiagnostics t3;
  qbat::BatchDiagnostics qbat;
  type25::BatchDiagnostics type25;
  type13::BatchDiagnostics type13;
  solids::BatchDiagnostics solids;
  bool has_qeph = false, has_t3 = false, has_qbat = false;
  bool has_type25 = false, has_type13 = false, has_solids = false;
  bool valid = false, kinetic_available = false;
};
struct ShellPhysicalPublicationIdentity {
  std::uint64_t configuration_id = 0, qualification_id = 0;
  ShellBatchStartup startup;
};
struct ShellPhysicalPublicationForecast {
  // Additional publisher-owned payload and its largest temporary proof phase.
  // Existing participant, physical ledger and rigid backings are shared, not
  // copied. Their reservations remain the composition caller's responsibility.
  std::size_t owned_host_bytes = 0, startup_host_bytes = 0;
  std::size_t device_bytes = 0;
};
} // namespace tl::fea
