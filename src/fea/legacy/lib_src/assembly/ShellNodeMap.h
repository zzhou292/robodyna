// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalNodeDomain.h"
#include "../elements/ShellBatchBinding.h"

namespace tl::fea {
struct ShellNodeMapLimits {
  std::size_t max_nodes=2048,max_host_bytes=8*1024*1024;
  static constexpr ShellNodeMapLimits Vehicle() noexcept {return {524288,1024*1024*1024};}
};
// Every active shell-local node maps to one exact source NID/position in the
// declared domain. Extra domain nodes are allowed; their mechanical coverage is
// another producer's obligation. Original shell connectivity/material inventory
// remains local and unchanged. No coefficient arithmetic or owner is added.
class ShellNodeMap {
 public:
  ShellNodeMap()=default;
  ShellNodeMap(const ShellNodeMap&) noexcept=default;
  ShellNodeMap(ShellNodeMap&& other) noexcept:ShellNodeMap(static_cast<const ShellNodeMap&>(other)) {}
  ShellNodeMap& operator=(const ShellNodeMap&)=delete;
  NodalDomainReport Initialize(const ShellBatchBinding&,const NodalNodeDomain&,ShellNodeMapLimits={}) noexcept;
  bool prepared() const noexcept {return bool(impl_);}
  const ShellBatchBinding* shells() const noexcept;
  const NodalNodeDomain* domain() const noexcept;
  std::size_t shell_node_count() const noexcept;
  std::size_t owner_node_count() const noexcept;
  tl::util::ConstView<std::size_t> mapping() const noexcept;
  std::size_t owner_index(std::size_t shell_node) const noexcept;
  bool identity_map() const noexcept; // Same extent and every index unchanged.
  bool Matches(const ShellBatchBinding&,const NodalNodeDomain&) const noexcept;
  bool Matches(const ShellNodeMap&) const noexcept;
  std::size_t owned_payload_bytes() const noexcept;
  std::size_t startup_payload_bytes() const noexcept;
 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};
} // namespace tl::fea
