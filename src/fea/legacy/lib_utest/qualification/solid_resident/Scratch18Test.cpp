// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include "ResultValues.h"
#include "../solid18_force/TestSupport.h"
#include <cstring>
#include <memory>

namespace solid_resident_test {
namespace {
using Traits = detail::Traits18;
using State = detail::State<Traits>;
void ExactState(const State& a, const State& b) {
  Exact(Traits::Read(a.history,a.cache),Traits::Read(b.history,b.cache));
  EXPECT_TRUE(tl::fea::solid18::detail::SameReference(a.history.reference(),b.history.reference()));
  EXPECT_TRUE(tl::fea::solid18::detail::SameMaterial(a.history.material(),b.history.material()));
}
TEST(SolidResidentScratch, ReusedPacketsPreserveConstructorAndFourHundredStepValues) {
  s::Parent18 parent;
  parent.reference = solid18_force_test::Reference();
  const auto material = solid18_force_test::Material();
  auto scratch = std::make_unique<detail::Scratch18>();
  State actual, expected;
  ASSERT_EQ(detail::InitializeState<Traits>(parent,material,{15.6464,0,0},expected),0);
  ASSERT_EQ(detail::InitializeState18(parent,material,{15.6464,0,0},*scratch,actual),0);
  ExactState(actual,expected);
  for (unsigned step = 0; step < 400; ++step) {
    SCOPED_TRACE(step);
    const auto interval = solid18_force_test::Path(parent.reference,step);
    State next;
    ASSERT_EQ(detail::UpdateState<Traits>(parent,material,expected,interval,next),0);
    ASSERT_EQ(detail::UpdateState18(parent,material,actual,interval,*scratch,actual),0);
    ExactState(actual,next);
    expected = next;
  }
  // Reusing a worker for a new parent must reset all constructor fields.
  parent.reference = solid18_force_test::Reference(false);
  ASSERT_EQ(detail::InitializeState<Traits>(parent,material,{},expected),0);
  ASSERT_EQ(detail::InitializeState18(parent,material,{},*scratch,actual),0);
  ExactState(actual,expected);
}
TEST(SolidResidentScratch, LastPointFailureLeavesStateAndSeparateWorkerUntouchedBeforeRetry) {
  s::Parent18 parent;
  parent.reference = solid18_force_test::Reference();
  const auto material = solid18_force_test::Material();
  auto scratch = std::make_unique<detail::Scratch18>();
  auto other = std::make_unique<detail::Scratch18>();
  State accepted, output, separate;
  ASSERT_EQ(detail::InitializeState18(parent,material,{},*scratch,accepted),0);
  ASSERT_EQ(detail::InitializeState18(parent,material,{1,2,3},*other,separate),0);
  const auto other_values = separate;
  const auto interval = solid18_force_test::Path(parent.reference,0);
  ASSERT_EQ(detail::UpdateState18(parent,material,accepted,interval,*scratch,output),0);
  const auto saved = output;
  unsigned char bytes[sizeof(State)];
  std::memcpy(bytes,&output,sizeof(output));
  auto values = accepted.history.data();
  values.point[7].material.point.stress_pa[0] = 1e308;
  State bad = accepted;
  ASSERT_EQ(tl::fea::solid18::PreparePrescribedHistory(parent.reference,material,values,
      accepted.history.stamp(),bad.history),tl::fea::solid18::Status::Success);
  EXPECT_NE(detail::UpdateState18(parent,material,bad,interval,*scratch,output),0);
  EXPECT_EQ(std::memcmp(bytes,&output,sizeof(output)),0);
  ExactState(separate,other_values);
  ASSERT_EQ(detail::UpdateState18(parent,material,accepted,interval,*scratch,output),0);
  ExactState(output,saved);
  auto invalid = interval;
  ++invalid.sample_index;
  EXPECT_NE(detail::UpdateState18(parent,material,accepted,invalid,*scratch,output),0);
  EXPECT_EQ(std::memcmp(bytes,&output,sizeof(output)),0);
}
TEST(SolidResidentScratch, ForecastChargesActualWorkersAndRejectsOneByteShort) {
  auto config = Config(372435);
  detail::ArenaLayout layout;
  const detail::Counts original{908,1309,195,1,1,law36_test::Curve.count};
  ASSERT_TRUE(detail::MakeLayout(original,config,layout));
  EXPECT_EQ(layout.scratch18.count,908u);
  EXPECT_EQ(layout.scratch18.bytes,908*sizeof(detail::Scratch18));
  config.limits.max_device_bytes = layout.bytes;
  detail::ArenaLayout exact;
  ASSERT_TRUE(detail::MakeLayout(original,config,exact));
  --config.limits.max_device_bytes;
  exact.bytes = 71;
  EXPECT_FALSE(detail::MakeLayout(original,config,exact));
  EXPECT_EQ(exact.bytes,71u);
  config = Config(9);
  ASSERT_TRUE(detail::MakeLayout({0,1,0,0,1,0},config,layout));
  EXPECT_EQ(layout.scratch18.count,0u);
  auto arena = std::make_unique<unsigned char[]>(layout.bytes);
  EXPECT_EQ(detail::RebasedHeader(arena.get(),layout).scratch18,nullptr);
}
TEST(SolidResidentScratch, GridStrideWorkspacesArePrivateForEveryAdmittedCount) {
  for (std::size_t parents : {std::size_t(0),std::size_t(1),std::size_t(63),
      std::size_t(64),std::size_t(65),std::size_t(908),detail::candidate_workers,
      detail::candidate_workers+1,std::size_t(16384)}) {
    const auto count = detail::Scratch18Count(parents);
    EXPECT_LE(count,detail::candidate_workers);
    for (std::size_t stride : {std::size_t(detail::candidate_threads),detail::candidate_workers}) {
      std::vector<unsigned> visits(parents,0);
      for (std::size_t worker = 0; worker < stride; ++worker) {
        for (std::size_t p = worker; p < parents; p += stride) {
          ASSERT_LT(worker,count);
          ++visits[p];
        }
      }
      for (auto n : visits) EXPECT_EQ(n,1u);
    }
  }
}
} // namespace
} // namespace solid_resident_test
