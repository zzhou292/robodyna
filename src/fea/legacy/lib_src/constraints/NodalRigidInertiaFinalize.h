// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalRigidGroupModel.h"
namespace tl::fea::rigid {
// Owning startup helper; exact legacy Eigen/Ispher2 order, no second producer.
NodalRigidGroupReport FinalizeInertia(const tl::math::Matrix3&,
    NodalRigidGroupProperties&,std::size_t group) noexcept;
}
