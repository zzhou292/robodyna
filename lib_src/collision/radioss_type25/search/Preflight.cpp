// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
namespace tlfea::contact::radioss_type25::search {
Status Maintenance::Preflight(const Source& source,Limits limits,Forecast& output) noexcept {
  detail::Layout layout;const auto status=detail::MakeLayout(source,limits,sizeof(Maintenance)+sizeof(Impl),layout);
  if(status==Status::Ok)output=layout.forecast;return status;
}
}
