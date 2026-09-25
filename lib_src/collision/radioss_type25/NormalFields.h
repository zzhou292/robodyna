// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "GeometryTypes.h"
#include <type_traits>
namespace tlfea::contact::radioss_type25::normal_fields {
// Shared value representation of native LBOUND / VTX_BISECTOR. Keep all raw
// float bits and the actual boundary value; no normalization or physical clock.
struct Reference {int boundary=0;StoredNormal bisector[2]{};};
static_assert(std::is_standard_layout<Reference>::value&&std::is_trivially_copyable<Reference>::value);
static_assert(sizeof(Reference)==sizeof(int)+2*sizeof(StoredNormal),"Preserve the existing normal-reference layout");
struct View {
  const StoredNormal* face_normals=nullptr;std::size_t normal_count=0; // Exactly four slots per main.
  const Reference* references=nullptr;std::size_t reference_count=0;
};
// All-zero View means legacy embedded fields. A present view must be complete;
// it never grants source, force-base, stream-completion or publication authority.
} // namespace tlfea::contact::radioss_type25::normal_fields
