// SPDX-License-Identifier: MIT
#include "Probes.h"
#include "lib_src/elements/ShellSourceRangeFilter.h"
namespace shell_output_range_test {
using fe::shell_source_range_detail::DisjointEnvelope;
TEST(ShellOutputEnvelope, OnlyWholeSpanSeparationProducesACertificate) {
  const auto first=Pointer(1000),last=Pointer(1100);
  EXPECT_TRUE(DisjointEnvelope(Pointer(900),100,first,16,last,8));
  EXPECT_TRUE(DisjointEnvelope(Pointer(1108),1,first,16,last,8));
  EXPECT_FALSE(DisjointEnvelope(Pointer(999),2,first,16,last,8));
  EXPECT_FALSE(DisjointEnvelope(Pointer(1107),2,first,16,last,8));
  EXPECT_FALSE(DisjointEnvelope(Pointer(1040),1,first,16,last,8)); // gap needs old scan
  EXPECT_TRUE(DisjointEnvelope(first,0,first,16,last,8));
  EXPECT_FALSE(DisjointEnvelope(Pointer(1001),0,first,16,last,8));
  EXPECT_TRUE(DisjointEnvelope(Pointer(1108),0,first,16,last,8));
  EXPECT_FALSE(DisjointEnvelope(Pointer(1150),1,first,200,last,8));
  EXPECT_TRUE(DisjointEnvelope(Pointer(1200),1,first,200,last,8));
}
TEST(ShellOutputEnvelope, MalformedEndpointsAndOutputOverflowNeverCertify) {
  const auto first=Pointer(1000),last=Pointer(1100),outside=Pointer(1);
  EXPECT_FALSE(DisjointEnvelope(outside,1,nullptr,16,last,8));
  EXPECT_FALSE(DisjointEnvelope(outside,1,first,16,nullptr,8));
  EXPECT_FALSE(DisjointEnvelope(outside,1,first,0,last,8));
  EXPECT_FALSE(DisjointEnvelope(outside,1,first,16,last,0));
  EXPECT_FALSE(DisjointEnvelope(outside,1,last,8,first,16));
  EXPECT_FALSE(DisjointEnvelope(outside,1,Pointer(UINTPTR_MAX-2),4,Pointer(UINTPTR_MAX-1),1));
  EXPECT_FALSE(DisjointEnvelope(outside,1,first,16,Pointer(UINTPTR_MAX-2),4));
  EXPECT_FALSE(DisjointEnvelope(nullptr,0,first,16,last,8));
  EXPECT_FALSE(DisjointEnvelope(nullptr,1,first,16,last,8));
  EXPECT_FALSE(DisjointEnvelope(Pointer(UINTPTR_MAX-2),4,first,16,last,8));
  EXPECT_TRUE(DisjointEnvelope(Pointer(UINTPTR_MAX-2),2,first,16,last,8));
}
}
