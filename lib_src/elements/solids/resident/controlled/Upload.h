// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../Arena.h"
namespace tl::fea::solids::batch_detail::controlled {
BatchReport Upload(const Model&,util::HostArena&,const ArenaLayout&,batch_detail::Storage&);
BatchReport Relocate(const Model&,batch_detail::Storage&)noexcept;
} // namespace tl::fea::solids::batch_detail::controlled
