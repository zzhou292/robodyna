// SPDX-License-Identifier: MIT
#pragma once
#include "Fixture.h"
namespace qbat_test {
std::array<double,40> NativeReference(const qb::ReferenceInput&);
std::array<double,84> NativeGeometry(const qb::CurrentInput&,int& native_flat_count);
} // namespace qbat_test
