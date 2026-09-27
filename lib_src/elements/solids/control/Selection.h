// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
namespace tl::fea::solids::control {
// Owns semantic selection and complete packet membership. App evidence owns
// file/binary/export authentication. No force/history, clock or runtime enable.
class Selection {
 public:
  Selection()=default;
  Selection(const Selection&) noexcept=default;
  Selection(Selection&& other) noexcept:impl_(other.impl_),initialized_(other.initialized_){}
  Selection& operator=(const Selection&)=delete;
  Selection& operator=(Selection&&)=delete;
  static Report Forecast(Input,std::size_t parent_count,Limits,Budget&) noexcept;
  Report Initialize(const SolidNodeContributions&,Input,Limits={}) noexcept;
  bool prepared()const noexcept{return initialized_;}
  Profile profile()const noexcept;
  std::uint64_t source_instance_id()const noexcept;
  UnitScale units()const noexcept;
  std::uint32_t native_nvsiz()const noexcept;
  std::uint32_t compiled_mvsiz()const noexcept;
  std::size_t controlled_count()const noexcept;
  util::ConstView<Parent> parents()const noexcept;
  util::ConstView<NativePartition> partitions()const noexcept;
  util::ConstView<Packet> packets()const noexcept;
  util::ConstView<Member> members()const noexcept;
  bool Matches(const Selection&)const noexcept;
  std::size_t owned_payload_bytes()const noexcept;
  std::size_t startup_payload_bytes()const noexcept;
 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
  bool initialized_=false;
};
} // namespace tl::fea::solids::control
