// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"
namespace extended_resident_test {
bool RearConstructor(const s::Parent18Law44&,const fe::solid18::law44::Material&,
    fe::solid18::Vec3,const s::Result18Law44&);
bool FoamConstructor(const s::Parent18Law90&,const tl::material::law90::PreparedMaterial&,
    const tl::material::law90::PreparationInput&,fe::solid18::Vec3,const s::Result18Law90&);
} // namespace extended_resident_test
