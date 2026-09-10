// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellBatchBinding.h"
#include "ShellBatchPlasticityBinding.h"

namespace tl::fea::shell_batch_detail {
// Vehicle forecasts count the embedded binding object through Impl. Shared
// inventory is discounted only when actual backing addresses AND extents match,
// never from equivalent bytes. Legacy conservative arithmetic stays unchanged.
// NodalMassBinding's opaque inclusive ledger is not discounted by this helper.
inline bool RetainedScopeBytes(const ShellBatchBinding* binding,const ShellBatchPlasticityBinding* catalog,
    bool vehicle,std::size_t& binding_bytes,std::size_t& catalog_bytes) noexcept {
  auto b=binding?binding->host_bytes():0,c=catalog?catalog->host_bytes():0;
  if(vehicle&&binding) {
    if(!binding->prepared()||b<sizeof(ShellBatchBinding))return false;
    b-=sizeof(ShellBatchBinding);
    if(catalog) {
      const auto& bi=binding->inventory();const auto& ci=catalog->inventory();
      if(bi.backing_bytes()&&bi.words().data()==ci.words().data()&&
          bi.words().size()==ci.words().size()&&bi.backing_bytes()==ci.backing_bytes()) {
        if(c<ci.backing_bytes())return false;
        c-=ci.backing_bytes();
      }
    }
  }
  binding_bytes=b;catalog_bytes=c;return true;
}
} // namespace tl::fea::shell_batch_detail
