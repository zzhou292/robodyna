#pragma once
#include "WallDerivedRead.h"
#include "WallJobAnalysisTestFixture.h"
#include "WallRawReportTestFixture.h"
#include <functional>

namespace tl::qualification::qeph::wall_recurrence::derived_test {
namespace io=crash::output;
namespace rt=raw_test;
// The raw map is an explicit free-particle/cache test operator, not native
// shell physics. Spectrum/Gram numbers are labelled synthetic measurements;
// these artifacts test the reviewed-producer protocol, never CW1 admission.
struct Fixture {
  rt::Directory directory;
  RawReadResult raw;
  RawReadBinding raw_binding;
  WallDerivedReadBinding binding;
  std::filesystem::path derived;
  explicit Fixture(bool partial=false,bool nonfinite=false);
};
using Mutation=std::function<void(const std::string&,io::Document&)>;
WallDerivedReadBinding Clone(const std::filesystem::path&,const std::filesystem::path&,const Mutation&);
std::string Snapshot(const WallDerivedReadResult&);
} // namespace tl::qualification::qeph::wall_recurrence::derived_test
