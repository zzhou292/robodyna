#pragma once
#include "lib_src/solvers/ExplicitNodalStep.h"
#include <gtest/gtest.h>
#include <vector>

namespace tl::fea::vehicle_test {
constexpr double H=1./1024;
struct Fields {
  std::size_t n;
  std::vector<double> x,v,w,q,rf,rc;
  explicit Fields(std::size_t count):n(count),x(3*n),v(3*n),w(3*n),q(4*n),rf(3*n),rc(3*n) {}
  NodalSnapshotBuffer buffer() { return {x.data(),v.data(),n,q.data(),w.data(),rf.data(),rc.data()}; }
  void Fill(double);
};
struct Initial {
  std::size_t n;
  std::vector<double> x,v,w,q,inverse,inertia;
  std::vector<std::uint8_t> fixed,rotation_fixed;
  explicit Initial(std::size_t);
  NodalStateConfig config() const;
  NodalReport Initialize(FENodalState&,const NodalStateConfig&) const;
};
class VehicleOwnerCuda:public ::testing::Test {
  void SetUp() override { int count=0; ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess); ASSERT_GT(count,0); }
};
void SparseLoads(const NodalAssemblyView&,bool bad_last=false);
void CorruptLastQuaternion(const NodalPreparedView&);
NodalReport Prepare(FENodalState&,NodalTrialToken&,bool bad_last=false);
void SameFields(const Fields&,const Fields&);
void InitialFields(const Initial&,const Fields&);
void Analytic(const Initial&,const Fields&,unsigned steps);
std::uint64_t FieldBitsHash(const Fields&);
} // namespace tl::fea::vehicle_test
