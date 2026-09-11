#pragma once
#include "PlacementFixture.h"
#include "lib_utest/qualification/shell_tab1_glass/Tab1NativeSupport.h"
namespace placement_test {
extern "C" void placement_native_shift(int,double*);
extern "C" void placement_native_rule(int,double*,double*,double*);
extern "C" void placement_native_coefficients(double,double,double,double,double,double,
    double,double,double,double,double,double*);
void NativePlacementStep(const sec::ShellLayeredJ2Input&,double,double,double,
    NativeState&,NativeTrace&);
inline void SameNativeState(const NativeState& a,const NativeState& b) {
  EXPECT_EQ(Bytes(a.points),Bytes(b.points));
  EXPECT_EQ(Bytes(a.failures),Bytes(b.failures));
  EXPECT_EQ(Bytes(a.material),Bytes(b.material));
  EXPECT_EQ(Bytes(a.stress),Bytes(b.stress));
  EXPECT_EQ(Bytes(a.moment),Bytes(b.moment));
  EXPECT_EQ(Bytes(a.work),Bytes(b.work));
  EXPECT_EQ(Bytes(a.thickness),Bytes(b.thickness));
  EXPECT_EQ(Bytes(a.parent),Bytes(b.parent));
}
} // namespace placement_test
