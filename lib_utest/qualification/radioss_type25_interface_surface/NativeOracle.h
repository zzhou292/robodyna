// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/radioss_type25/surface_interface/Types.h"
#include <vector>
namespace type25_interface_surface_test {
namespace n=tlfea::contact::radioss_type25;
namespace f=n::surface_interface;
namespace s=n::startup;
struct NativeResult {
  std::vector<s::PrimaryFace> primary;
  std::vector<s::PrimaryFaceIdentity> identities,raw_origins;
  std::vector<f::RawClassification> classifications;
  std::vector<std::uint32_t> raw_to_primary,primary_to_raw,expanded_to_primary,primary_to_partner;
  std::vector<std::uint8_t> surface_solid_flags;
  std::vector<s::Main> mains;
  std::size_t shell_primary_count=0;
};
// Complete original IN24/filter/SH2, bounded qualification only, serial COMMON.
// Source IDs/origin availability are wrapper metadata, not native force outputs.
NativeResult Oracle(const f::Input&);
}
