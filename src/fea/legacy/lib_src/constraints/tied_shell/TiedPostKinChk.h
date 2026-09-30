// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TiedClassification.h"

namespace tl::constraints::tied_shell {
// Caller assertion, not source authentication. This profile excludes all
// KINCHK branches which permanently modify IKINE or a decode table. It does
// not specify unobserved rigid hierarchy warnings or initialize velocities.
enum class KinChkProfile { Unspecified, NoWallRbeOrCyclic };
struct KinChkSlave {
  std::uint32_t source_id = 0;
  std::int32_t irupt = 0; // Preserved association: KINCHK never consumes IRUPT.
  NativeKinematics kinematics;
};
struct PostKinChkSlave {
  KinChkSlave before;
  std::int32_t kinet = 0;
  // Native possible-conflict predicates on this observed node only. They do
  // not switch CIN/PEN and are not a complete starter warning count.
  bool repeated_condition = false;
  bool mixed_incompatible_conditions = false;
};
struct KinChkInput {
  KinChkProfile profile = KinChkProfile::Unspecified;
  ClassificationPhase phase = ClassificationPhase::Empty;
  std::uint64_t source_instance_id = 0;
  std::uint32_t source_interface_id = 0;
  ClassificationView<KinChkSlave> slaves;
  ClassificationView<std::int32_t> interface_decode;
};
struct KinChkLimits {
  std::size_t max_slaves = 65536;
  std::size_t max_host_bytes = 64u << 20;
};
struct KinChkForecast {
  std::size_t owned_payload_bytes = 0;
  std::size_t startup_payload_bytes = 0;
};
class PostKinChkResult {
 public:
  PostKinChkResult() noexcept = default;
  ClassificationView<PostKinChkSlave> slaves() const noexcept;
  ClassificationView<std::int32_t> interface_decode() const noexcept;
  std::uint64_t source_instance_id() const noexcept;
  std::uint32_t source_interface_id() const noexcept;
  KinChkForecast forecast() const noexcept;
  bool SharesStorage(const PostKinChkResult& other) const noexcept { return data_ && data_ == other.data_; }
 private:
  struct Data;
  std::shared_ptr<const Data> data_;
  friend ClassificationReport ForecastPostKinChk(std::size_t, std::size_t, KinChkForecast*, KinChkLimits) noexcept;
  friend ClassificationReport PostKinChk(const KinChkInput&, PostKinChkResult*, KinChkLimits) noexcept;
};
// Both functions check count/byte limits before borrowed payload reads. Old
// result payload is charged during replacement; input may borrow that result.
// Allocation/control-block overhead and RSS are outside these payload budgets.
ClassificationReport ForecastPostKinChk(std::size_t slaves, std::size_t old_payload_bytes,
    KinChkForecast*, KinChkLimits = {}) noexcept;
ClassificationReport PostKinChk(const KinChkInput&, PostKinChkResult*, KinChkLimits = {}) noexcept;
} // namespace tl::constraints::tied_shell
