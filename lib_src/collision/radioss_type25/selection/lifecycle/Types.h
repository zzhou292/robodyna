// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../GeometryTypes.h"
#include "../../CoefficientTypes.h"
#include "../../Units.h"
#include "lib_src/collision/SurfaceContactTypes.h"
#include "../Types.h"
#include <cstddef>
#include <cstdint>
namespace tlfea::contact::radioss_type25::selection::lifecycle {
struct Node {
  std::uint64_t source_id=0;
  int constraint=0,skew=0; // Original ICODT / ISKEW.
};
struct Main {
  int global_id=0,segment_type=0; // MSEGLO / signed MSEGTYP.
  std::uint32_t nodes[4]{};
  StoredNormal normal_slot[4]{};
  int normal_reference[4]{},neighbors[4]{}; // ADMSR / MVOISIN.
  double coefficient=0,gap[4]{},maximum_gap=0;
  // Actual local main index is array position+1; no implicit side generation.
};
struct Secondary {
  std::uint32_t node=0;
  double coefficient=0,gap=0;
  int initial_contact_flag=0; // Incoming runtime ICONT_I; persist staged row output.
};
struct NormalReference {
  int boundary=0; // Original LBOUND zero/nonzero value.
  StoredNormal bisector[2]{};
};
// Exact native prefix sums, represented in zero-based C++ storage. Main entries
// retain their original positive local IDs and complete source incidence order.
struct Csr {
  const std::uint32_t* offsets=nullptr;
  std::size_t offset_count=0;
  const std::uint32_t* entries=nullptr;
  std::size_t entry_count=0;
};
struct SourceView {
  const Node* nodes=nullptr;std::size_t node_count=0;
  const Main* mains=nullptr;std::size_t main_count=0; // CLASSIFICATION NRTM.
  const Secondary* secondary=nullptr;std::size_t secondary_count=0;
  const NormalReference* normals=nullptr;std::size_t normal_count=0;
  Csr normal_to_main,removed_main_by_secondary;
  std::uint64_t generation=0;
  // This numerical view never authenticates a physical source roster or clock.
};
struct Profile {
  selection::Profile selection;
  GeometryProfile geometry;
  PairCoefficientProfile coefficient;
  double minimum_coefficient=0,maximum_coefficient=0;
  int neighbor_removal=-1; // Selected FLAGREMN=2.
  int optcd_response_precision=-1; // Explicit raw IRESP0/1/2; 1 uses special PREC, 0/2 use EM8; -1 unknown rejects.
};
struct Step {
  double time=0,previous_dt=0; // Supplied native TT / DT1, never an owned clock.
};
// Complete raw search inventory, before original local OPTCD filtering.
struct SpatialOccurrence {
  int secondary=0,local_main=0; // Original positive native CAND_N / CAND_E.
};
enum class KinematicsUnits { Native, Si };
struct Kinematics {
  VectorView positions,velocities; // Borrowed accepted X_n and native V-stage.
  KinematicsUnits units=KinematicsUnits::Native;
  UnitScale native_units; // Explicit only when converting SI X/V to native.
};
struct Input {
  Profile profile;
  Step step;
  SourceView source;
  Kinematics current;
  const NativeGeometryHistory* accepted_rows=nullptr;
  std::size_t accepted_row_count=0;
  const SpatialOccurrence* spatial=nullptr;std::size_t spatial_count=0;
  Csr spatial_by_secondary; // Entries are zero-based original spatial ordinals.
};
enum class Origin { Retained, Spatial, Sliding };
struct SelectedGeometry {
  bool enabled=false;
  GeometryRowKey key;
  int local_main=0,subtriangle=0,selection_code=0; // Preserve complete final IRTLM(2).
  double lb=0,lc=0,incoming_stiffness=0;
  // Bind on demand to current source nodes/normals/gaps. Never copy the
  // original occurrence's geometry when its chosen cache main was rewritten.
};
struct Occurrence {
  bool cache_initialized=false; // Must be set by a complete native classification stage.
  int secondary=0,local_main=0; // KEEPF may negate secondary.
  Origin origin=Origin::Spatial;
  std::size_t source_ordinal=0; // Retained row / spatial ordinal / row-local append.
  // cache.occurrence is the FINAL global ordinal: retained prefix, unchanged
  // spatial order, then secondary/append order. GPU row scheduling cannot alter it.
  CandidateCache cache;
  SelectedGeometry selected;
};
struct RowResult {
  NativeGeometryHistory history;
  int initial_contact_flag=0; // Staged OPTCD transition; not reset from Starter each step.
  int sliding_reference[4]{};
  std::size_t retained_count=0,optimized_count=0,sliding_count=0;
  std::size_t continuation_count=0,new_impact_count=0,kept_count=0;
  std::size_t zero_sum_resets=0;
};
enum class Stage { Admission,Begin,Optimize,Retained,Sliding,Continuation,NewImpact,Keep,GeometryBinding,Finish };
struct Report {
  selection::Status status=selection::Status::InvalidInput;
  Stage stage=Stage::Admission;
  std::size_t secondary=SIZE_MAX,occurrence=SIZE_MAX;
  std::size_t required_candidates=0;
  bool count_complete=false;
};
struct Limits {
  std::size_t rows=0,candidates=0,sliding_scratch=0;
  std::size_t scratch_bytes=0;
};
} // namespace tlfea::contact::radioss_type25::selection::lifecycle
