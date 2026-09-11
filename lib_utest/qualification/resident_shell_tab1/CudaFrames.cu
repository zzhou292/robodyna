#include "CudaFixture.h"
#include "../shell_failure_force/NativeAgreement.h"

namespace resident_tab1_test {
namespace {
template<class Batch, class Force, class Diagnostic>
bool ReadFamily(Batch& batch, const fe::NodalStamp& stamp, const Diagnostic* candidate,
    std::array<Force, Parents>& forces, std::array<fe::ShellBatchLayeredSection, Parents>& sections,
    std::array<fe::ShellBatchFailureState, Parents>& failure, Diagnostic& diagnostic) {
  auto report = candidate ? batch.CopyPreparedResults(*candidate, forces.data(), Parents) :
      batch.CopyAcceptedResults(stamp, forces.data(), Parents, &diagnostic);
  using Status = decltype(report.status);
  EXPECT_EQ(report.status, Status::Success) << report.message;
  if (report.status != Status::Success) return false;
  if (candidate) diagnostic = *candidate;
  Diagnostic other;
  report = candidate ? batch.CopyPreparedLayeredSectionHistory(*candidate, sections.data(), Parents) :
      batch.CopyAcceptedLayeredSectionHistory(stamp, sections.data(), Parents, &other);
  EXPECT_EQ(report.status, Status::Success) << report.message;
  if (report.status != Status::Success) return false;
  report = candidate ? batch.CopyPreparedFailureHistory(*candidate, failure.data(), Parents) :
      batch.CopyAcceptedFailureHistory(stamp, failure.data(), Parents, &other);
  EXPECT_EQ(report.status, Status::Success) << report.message;
  return report.status == Status::Success;
}
void SameSection(const fe::ShellBatchLayeredSection& a, const fe::ShellBatchLayeredSection& b) {
  ASSERT_EQ(a.law(), b.law());
  if (const auto* elastic = a.elastic()) {
    ASSERT_NE(b.elastic(), nullptr);
    std::vector<double> x, y;
    for (unsigned p = 0; p < 3; ++p) {
      failure_force_test::Append(x, elastic->point[p].stress);
      failure_force_test::Append(y, b.elastic()->point[p].stress);
    }
    placed::Exact(x, y);
  } else {
    ASSERT_NE(a.plastic(), nullptr);
    ASSERT_NE(b.plastic(), nullptr);
    placed::Exact(Values(*a.plastic()), Values(*b.plastic()));
  }
}
} // namespace
bool Read(Rig& rig, Frame& output, const fe::ShellBatchDiagnostics* candidate) {
  Frame next;
  if (candidate) next.diagnostics = *candidate;
  if (!ReadFamily(rig.qeph, rig.owner.accepted(), candidate ? &candidate->qeph : nullptr,
        next.qforce, next.qsection, next.qfailure, next.diagnostics.qeph) ||
      !ReadFamily(rig.t3, rig.owner.accepted(), candidate ? &candidate->t3 : nullptr,
        next.tforce, next.tsection, next.tfailure, next.diagnostics.t3)) return false;
  output = next;
  return true;
}
void Same(const Frame& a, const Frame& b) {
  for (unsigned e = 0; e < Parents; ++e) {
    placed::Exact(placed::ForceValues(a.qforce[e]), placed::ForceValues(b.qforce[e]));
    placed::Exact(placed::ForceValues(a.tforce[e]), placed::ForceValues(b.tforce[e]));
    placed::Exact(failure_force_test::Diagnostics(a.qforce[e].diagnostics), failure_force_test::Diagnostics(b.qforce[e].diagnostics));
    placed::Exact(failure_force_test::Diagnostics(a.tforce[e].diagnostics), failure_force_test::Diagnostics(b.tforce[e].diagnostics));
    SameSection(a.qsection[e], b.qsection[e]);
    SameSection(a.tsection[e], b.tsection[e]);
    placed::Exact(Values(a.qfailure[e]), Values(b.qfailure[e]));
    placed::Exact(Values(a.tfailure[e]), Values(b.tfailure[e]));
  }
}
} // namespace resident_tab1_test
