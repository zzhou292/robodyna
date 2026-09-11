// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Arena.h"

namespace tl::fea::solids::batch_detail {
BatchReport UploadMaterials(const Model&, util::HostArena&, const ArenaLayout&);
solid18::Material ExpectedMaterial36(const Model&, std::size_t, double*) noexcept;
solid18::law44::Material ExpectedMaterial44(const Model&, std::size_t, double*) noexcept;
BatchReport ExpectedMaterial90(const Model&, std::size_t, double*,
    solid18::total_strain::Material&) noexcept;
} // namespace tl::fea::solids::batch_detail
