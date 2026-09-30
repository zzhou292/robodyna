// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TiedPostKinChk.h"
#include "../../../lib_utils/BoundedArena.h"
#include "../../../lib_utils/SourceIdentityIndex.h"
#include <array>
#include <climits>

namespace tl::constraints::tied_shell {
struct PostKinChkResult::Data {
  std::uint64_t source = 0;
  std::uint32_t interface = 0;
  std::size_t count = 0;
  KinChkForecast bytes;
  std::unique_ptr<PostKinChkSlave[]> slaves;
  std::array<std::int32_t,8192> decode{};
};
namespace {
using Index = tl::util::SourceIdentityIndex<1>;
using Status = ClassificationStatus;
ClassificationReport Fail(Status status, std::size_t row = SIZE_MAX) noexcept {
  return {status,row,SIZE_MAX};
}
template<class T> bool Span(ClassificationView<T> value) noexcept {
  const auto address = reinterpret_cast<std::uintptr_t>(value.data);
  return value.data && address % alignof(T) == 0 && value.count <= SIZE_MAX/sizeof(T) &&
         address <= UINTPTR_MAX-value.count*sizeof(T);
}
bool Mask(std::int32_t code) noexcept {
  // RWALL, RBE2 and RBE3 tables are excluded throughout this supplied context.
  return code >= 0 && code < 8192 && (code & (4|2048|4096)) == 0;
}
bool Directions(std::int32_t code) noexcept { return code >= 0 && code%10 <= 7; }
int Decode(int kind, int code, const std::int32_t* table) noexcept {
  return kind == 2 ? table[code] : ((code & kind) != 0);
}
PostKinChkSlave Observe(const KinChkSlave& slave, const std::int32_t* table) noexcept {
  PostKinChkSlave result;
  result.before = slave;
  result.kinet = slave.kinematics.conditions;
  const auto conditions = slave.kinematics.conditions;
  const auto duplicate = slave.kinematics.duplicate_conditions;
  for (int kind = 1; kind <= 4096; kind *= 2) {
    if (Decode(kind,conditions,table) == 1 && Decode(kind,duplicate,table) == 1)
      result.repeated_condition = true;
  }
  // With no wall bit, the native MARQUEUR branch is exactly the ordinary
  // nonsingle-condition branch. Native warnings do not change these values.
  const bool single = conditions == 0 || (conditions & (conditions-1)) == 0;
  result.mixed_incompatible_conditions = !single && slave.kinematics.incompatible_conditions != 0;
  return result;
}
}
ClassificationReport ForecastPostKinChk(std::size_t count, std::size_t old_bytes,
    KinChkForecast* output, KinChkLimits limits) noexcept {
  if (!output) return Fail(Status::InvalidInput);
  if (count > limits.max_slaves || count > INT_MAX/64) return Fail(Status::ResourceLimit);
  if (!count) return Fail(Status::InvalidInput);
  tl::util::BoundedArenaLayout layout(limits.max_host_bytes);
  tl::util::ArenaRegion region;
  if (!layout.Append<unsigned char>(sizeof(PostKinChkResult::Data),region) ||
      !layout.Append<PostKinChkSlave>(count,region))
    return Fail(Status::ResourceLimit);
  KinChkForecast next{layout.bytes(),0};
  if (!layout.Append<unsigned char>(old_bytes,region) ||
      !layout.Append<Index::Entry>(count,region) || !layout.Append<unsigned char>(sizeof(Index),region))
    return Fail(Status::ResourceLimit);
  next.startup_payload_bytes = layout.bytes();
  *output = next;
  return {};
}
ClassificationReport PostKinChk(const KinChkInput& input, PostKinChkResult* output, KinChkLimits limits) noexcept {
  if (!output) return Fail(Status::InvalidInput);
  KinChkForecast bytes;
  auto report = ForecastPostKinChk(input.slaves.count,output->forecast().owned_payload_bytes,&bytes,limits);
  if (!report) return report;
  if (input.profile != KinChkProfile::NoWallRbeOrCyclic ||
      input.phase != ClassificationPhase::InterfaceTaggedBeforeKinChk ||
      !input.source_instance_id || !input.source_interface_id || input.interface_decode.count != 8192 ||
      !Span(input.slaves) || !Span(input.interface_decode))
    return Fail(Status::InvalidInput);
  for (std::size_t word = 0; word < 8192; ++word) {
    const auto value = input.interface_decode.data[word];
    // ITAGSL2 only clears initial KININI interface entries.
    if (value < 0 || value > ((word & 2) != 0)) return Fail(Status::NativeDomain,word);
  }
  try {
    Index index;
    index.Prepare(input.slaves.count,[&](std::size_t row) { return input.slaves.data[row].source_id; });
    auto next = std::make_shared<PostKinChkResult::Data>();
    next->slaves = std::make_unique<PostKinChkSlave[]>(input.slaves.count);
    for (std::size_t row = 0; row < input.slaves.count; ++row) {
      const auto& slave = input.slaves.data[row];
      const auto& k = slave.kinematics;
      if (!slave.source_id || slave.source_id > INT_MAX || (slave.irupt != 0 && slave.irupt != 1))
        return Fail(Status::InvalidInput,row);
      if (index.First(slave.source_id) != row) return Fail(Status::DuplicateIdentity,row);
      if (!Mask(k.conditions) || !Mask(k.duplicate_conditions) || !Mask(k.incompatible_conditions) ||
          !Directions(k.translation) || !Directions(k.rotation)) return Fail(Status::NativeDomain,row);
      next->slaves[row] = Observe(slave,input.interface_decode.data);
    }
    next->source = input.source_instance_id;
    next->interface = input.source_interface_id;
    next->count = input.slaves.count;
    next->bytes = bytes;
    for (std::size_t word = 0; word < 8192; ++word) next->decode[word] = input.interface_decode.data[word];
    output->data_ = std::move(next);
    return {};
  } catch (...) {
    return Fail(Status::ResourceLimit);
  }
}
ClassificationView<PostKinChkSlave> PostKinChkResult::slaves() const noexcept {
  return data_ ? ClassificationView<PostKinChkSlave>{data_->slaves.get(),data_->count} : ClassificationView<PostKinChkSlave>{};
}
ClassificationView<std::int32_t> PostKinChkResult::interface_decode() const noexcept {
  return data_ ? ClassificationView<std::int32_t>{data_->decode.data(),data_->decode.size()} : ClassificationView<std::int32_t>{};
}
std::uint64_t PostKinChkResult::source_instance_id() const noexcept { return data_ ? data_->source : 0; }
std::uint32_t PostKinChkResult::source_interface_id() const noexcept { return data_ ? data_->interface : 0; }
KinChkForecast PostKinChkResult::forecast() const noexcept { return data_ ? data_->bytes : KinChkForecast{}; }
} // namespace tl::constraints::tied_shell
