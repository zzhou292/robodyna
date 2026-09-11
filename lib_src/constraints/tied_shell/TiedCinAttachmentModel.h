// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TiedPostKinChk.h"
#include "TiedPatchGeometry.h"
#include "../../assembly/NodalNodeDomain.h"
#include <array>

namespace tl::constraints::tied_shell {
enum class CinMasterTopology { Quad, TriangleRepeatedThird };
enum class CinMasterSourceKind { DeclaredShellElement };
struct CinMasterSource {
  CinMasterSourceKind kind = CinMasterSourceKind::DeclaredShellElement;
  std::uint64_t element_id = 0, part_id = 0;
};
struct CinAttachmentDeclaration {
  std::uint32_t original_nsv_row = 0;
  std::uint64_t ordered_master_rank = 0; // Original one-based IRECT rank.
  CinMasterSource master_source;
  CinMasterTopology topology = CinMasterTopology::Quad;
  std::uint64_t secondary_source_id = 0;
  std::array<std::uint64_t,4> master_source_ids{};
  // Original represented SI positions: secondary, then four ordered masters.
  std::array<Vec3,5> reference_positions{};
};
struct CinAttachmentRow {
  std::uint32_t original_nsv_row = 0;
  std::uint64_t ordered_master_rank = 0;
  CinMasterSource master_source;
  CinMasterTopology topology = CinMasterTopology::Quad;
  std::uint32_t secondary_domain_node = 0;
  std::array<std::uint32_t,4> master_domain_nodes{};
  Patch reference_patch; // Reference assessment only; not current DPARA.
};
struct CinAttachmentLimits {
  std::size_t max_attachments = 65536;
  std::size_t max_host_bytes = 64u << 20;
};
struct CinAttachmentForecast {
  // Immutable backing only; the copied domain/result handles live in Data.
  std::size_t domain_payload_bytes = 0, post_kinchk_payload_bytes = 0;
  std::size_t model_payload_bytes = 0, scratch_bytes = 0;
  std::size_t startup_payload_bytes = 0;
};
enum class CinAttachmentStatus {
  Success, InvalidInput, ResourceLimit, SourceMismatch, MissingDomainNode,
  PositionMismatch, UnsupportedDisposition, ConflictingMaster, InvalidTopology, ReferencePatchRejected
};
struct CinAttachmentReport {
  CinAttachmentStatus status = CinAttachmentStatus::Success;
  std::size_t row = SIZE_MAX, slot = SIZE_MAX;
  Status patch_status = Status::Success;
  explicit operator bool() const noexcept { return status == CinAttachmentStatus::Success; }
};
enum class CinAttachmentObligation { Pending };
class TiedCinAttachmentModel {
 public:
  TiedCinAttachmentModel() noexcept = default;
  bool prepared() const noexcept { return bool(data_); }
  const fea::NodalNodeDomain* domain() const noexcept;
  const PostKinChkResult* classification() const noexcept;
  ClassificationView<CinAttachmentRow> rows() const noexcept;
  CinAttachmentForecast forecast() const noexcept;
  CinAttachmentObligation current_geometry() const noexcept { return CinAttachmentObligation::Pending; }
  CinAttachmentObligation master_activity_and_release() const noexcept { return CinAttachmentObligation::Pending; }
 private:
  struct Data;
  std::shared_ptr<const Data> data_;
  friend CinAttachmentReport ForecastCinAttachments(const PostKinChkResult&,const fea::NodalNodeDomain&,
      std::size_t,std::size_t,CinAttachmentForecast*,CinAttachmentLimits) noexcept;
  friend CinAttachmentReport PrepareCinAttachments(const PostKinChkResult&,const fea::NodalNodeDomain&,
      ClassificationView<CinAttachmentDeclaration>,TiedCinAttachmentModel*,CinAttachmentLimits) noexcept;
};
// Exact domain/source identity only. This does not produce physical coefficients,
// current kinematics, a failure-release policy, constrained DOFs or an owner.
// One complete CIN scope is required: penalty/conflicting rows are not dropped.
// Budgets precede borrowed reads; failure preserves an old result and old inputs.
CinAttachmentReport ForecastCinAttachments(const PostKinChkResult&, const fea::NodalNodeDomain&,
    std::size_t count,std::size_t old_distinct_retained_bytes,CinAttachmentForecast*,CinAttachmentLimits = {}) noexcept;
CinAttachmentReport PrepareCinAttachments(const PostKinChkResult&, const fea::NodalNodeDomain&,
    ClassificationView<CinAttachmentDeclaration>,TiedCinAttachmentModel*,CinAttachmentLimits = {}) noexcept;
}
