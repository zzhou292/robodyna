// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_utest/qualification/self_contact_current_regularity/Fixture.h"
#include "lib_utest/qualification/self_contact_filter_batch/Corpus.h"
#include "lib_src/collision/self_contact_transaction/Storage.h"
#include <string_view>

namespace facet_filter_benchmark {
namespace c = tlfea::contact;
namespace sct = c::self_contact_transaction;
namespace f = c::self_contact_filters;
struct Row {
  sct::PairMotionAction action = sct::PairMotionAction::UnsupportedRigidArc;
  f::PairResult numerical;
};
struct Scene {
  // Existing authenticated small binding fixture, constructed once outside
  // timing. GTest is confined to this qualification executable, never the CLI.
  current_regularity_test::Fixture source{0, true};
  std::vector<c::FixedContactFacet> descriptors;
  std::vector<c::CurrentFixedTriangle> accepted, prepared;
  std::vector<sct::MotionSupport> motion;
  std::vector<c::SelfContactSweptParentBounds> bounds;
  std::vector<c::FixedTrianglePair> pairs;
  std::size_t linear_rows = 0, nonlinear_rows = 0, excluded_rows = 0;
  void Initialize(std::size_t count, std::string_view pattern);
  Row Scalar(std::size_t ordinal) const;
};
void Require(bool, const char*);
}  // namespace facet_filter_benchmark
