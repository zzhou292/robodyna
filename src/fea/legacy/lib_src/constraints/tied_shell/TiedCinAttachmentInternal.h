// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TiedCinAttachmentModel.h"
#include "../../assembly/NodalDomainIdentity.h"
#include "../../../lib_utils/BoundedArena.h"
#include "../../../lib_utils/SourceIdentityIndex.h"

namespace tl::constraints::tied_shell {
struct TiedCinAttachmentModel::Data {
  Data(const PostKinChkResult& p,const fea::NodalNodeDomain& d) : post(p),domain(d) {}
  PostKinChkResult post;
  fea::NodalNodeDomain domain;
  std::unique_ptr<CinAttachmentRow[]> rows;
  std::size_t count = 0;
  bool explicitly_empty = false;
  CinAttachmentForecast bytes;
};
namespace cin_detail {
using Report = CinAttachmentReport;
using ResultStatus = CinAttachmentStatus;
using Index = util::SourceIdentityIndex<1>;
// Match the immutable node-domain convention. This conservative reservation
// covers shared control, without claiming an allocator/RSS prediction.
inline constexpr std::size_t SharedControlReserveBytes = 64;
inline Report Fail(ResultStatus status,std::size_t row = SIZE_MAX,std::size_t slot = SIZE_MAX) noexcept {
  return {status,row,slot,Status::Success};
}
Report Map(const CinAttachmentDeclaration&,const PostKinChkSlave&,const fea::NodalNodeDomain&,
           std::size_t row,CinAttachmentRow&) noexcept;
bool SameMaster(const CinAttachmentRow&,const CinAttachmentRow&) noexcept;
}
}
