// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "Oracle.h"
#include "ResultAssertions.h"
#include "lib_src/collision/represented_interval_crossing/NativeStorageQualification.h"
#include "lib_src/collision/represented_interval_crossing/FixedPolicyQualification.h"
#include <cmath>
#include <limits>

namespace represented_interval_test {
namespace zero_storage {
namespace native = ct::represented_interval_crossing;
using Paths = std::array<ct::RepresentedTrianglePath, 2>;
inline void SameProjection(const native::ExactProjectionDomainReport& a,
    const native::ExactProjectionDomainReport& b) {
  EXPECT_EQ(std::tie(a.supported,a.eligible,a.sample_depth,a.coordinate_bits,
      a.degree_two_bits,a.limb_bits,a.karatsuba_cutoff),
      std::tie(b.supported,b.eligible,b.sample_depth,b.coordinate_bits,
      b.degree_two_bits,b.limb_bits,b.karatsuba_cutoff));
}
inline native::NativeStorageComparison Compare(const Paths& paths,
    ct::RepresentedIntervalLimits limits = {}) {
  const auto result = native::CompareNativeStorage(paths[0], paths[1], limits);
  EXPECT_EQ(result.status, S::Ok);
  SameResult(result.original.result, result.current.result);
  const auto fixed = native::CompareFixedIntegerPolicy(paths[0], paths[1], limits);
  EXPECT_EQ(fixed.status, S::Ok);
  SameResult(result.original.result, fixed.current);
  SameProjection(result.domain.projection,
      native::ExactProjectionDomain::FromPaths(paths[0], paths[1], limits.max_depth).report());
  EXPECT_EQ(result.current.counters.narrow_pairs, result.domain.eligible ? 1u : 0u);
  EXPECT_EQ(result.original.counters.wide_pairs, 1u);
  return result;
}

TEST(RepresentedZeroNeutralStorage, OriginalZeroCoordinateFamiliesKeepWitnessAndWorkAtAllDepths) {
  const auto first = Static(10, BaseTriangle());
  const std::array<ct::Vec3,3> point{};
  for (const auto& second : {Static(20,BaseTriangle()),Static(20,BaseTriangle(1)),
      Path(20,BaseTriangle(1),BaseTriangle(-3)),
      Path(20,BaseTriangle(1),BaseTriangle(1-4096)),
      Static(20,{{{2,0,0},{3,0,0},{2,-1,0}}}),Static(20,point)}) {
    for (unsigned depth : {0u,20u,52u}) for (std::size_t work : {1u,31u}) {
      SCOPED_TRACE(depth); SCOPED_TRACE(work);
      ct::RepresentedIntervalLimits limits;limits.max_depth=depth;limits.max_work_per_pair=work;
      const auto result=Compare({first,second},limits);
      ASSERT_TRUE(result.domain.eligible);
      EXPECT_LE(result.domain.storage.coordinate_bits,117u);
      EXPECT_GT(result.domain.projection.coordinate_bits,125u);
      if(result.current.result.classification==C::CertifiedCrossingContact) {
        const auto& witness=result.current.result;
        const auto exact=ExactOracleAt(first,second,witness.witness_time_numerator,
            std::uint64_t{1}<<witness.witness_time_depth);
        EXPECT_TRUE(exact.valid && exact.intersects);
      }
    }
  }
}
TEST(RepresentedZeroNeutralStorage, SignedAndAllZeroCoordinatesRetainDegenerateMeaning) {
  std::array<ct::Vec3,3> zero{};
  const auto allzero=Compare({Static(10,zero),Static(20,zero)});
  ASSERT_TRUE(allzero.domain.eligible);
  EXPECT_EQ(allzero.current.result.classification,C::Unresolved);
  EXPECT_EQ(allzero.current.result.reason,R::DegenerateGeometry);
  auto negative=BaseTriangle();for(auto& p:negative)p.z=-0.;
  const auto plus=Compare({Static(10,BaseTriangle()),Static(20,BaseTriangle())});
  const auto minus=Compare({Static(10,BaseTriangle()),Static(20,negative)});
  ASSERT_TRUE(minus.domain.eligible);
  SameResult(plus.current.result,minus.current.result);
}
TEST(RepresentedZeroNeutralStorage, NonzeroBoundaryStillControls125Versus126Bits) {
  for(unsigned depth:{0u,20u,52u})for(unsigned bits:{125u,126u}) {
    std::array<ct::Vec3,3> triangle{{{2,2,2},{4,2,0},{2,4,2}}};
    triangle[0].z=std::ldexp(1.,55+int(depth+1)-int(bits));
    ct::RepresentedIntervalLimits limits;limits.max_depth=depth;
    const auto result=Compare({Static(10,triangle),Static(20,triangle)},limits);
    EXPECT_EQ(result.domain.storage.coordinate_bits,bits);
    EXPECT_GT(result.domain.projection.coordinate_bits,125u);
    EXPECT_EQ(result.domain.eligible,bits==125);
  }
}
TEST(RepresentedZeroNeutralStorage, HugeAndSubnormalScalesDoNotBroadenProjectionShortcuts) {
  for(int exponent:{-1070,-1000,0,700,1020}) {
    auto a=BaseTriangle(),b=BaseTriangle(1);
    for(auto* triangle:{&a,&b})for(auto& p:*triangle) {
      p.x=std::ldexp(p.x,exponent);p.y=std::ldexp(p.y,exponent);p.z=std::ldexp(p.z,exponent);
    }
    const auto result=Compare({Static(10,a),Static(20,b)});
    ASSERT_TRUE(result.domain.eligible);
    EXPECT_EQ(result.current.result.classification,C::CertifiedSeparated);
    if(exponent>=700) EXPECT_FALSE(result.domain.projection.eligible);
  }
  auto mixed=BaseTriangle();mixed[0].z=std::numeric_limits<double>::denorm_min();
  const auto wide=Compare({Static(10,mixed),Static(20,mixed)});
  EXPECT_FALSE(wide.domain.eligible);
  EXPECT_GT(wide.domain.storage.coordinate_bits,125u);
}
TEST(RepresentedZeroNeutralStorage, NonfiniteAndUnsupportedPathsNeverObtainZeroAdmission) {
  auto paths=Paths{Static(10,BaseTriangle()),Static(20,BaseTriangle())};
  for(double bad:{std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
    auto invalid=paths;invalid[1].vertices[2].endpoint[1].z=bad;
    EXPECT_FALSE(native::NativeStorageDomain::FromPaths(invalid[0],invalid[1],20).eligible());
    const auto compared=native::CompareNativeStorage(invalid[0],invalid[1],{});
    EXPECT_NE(compared.status,S::Ok);
    EXPECT_EQ(compared.current.counters.narrow_pairs+compared.current.counters.wide_pairs,0u);
  }
  paths[1].motion=ct::RepresentedMotion::RigidArc;
  const auto unsupported=Compare(paths);
  EXPECT_FALSE(unsupported.domain.eligible);
  EXPECT_EQ(unsupported.current.result.reason,R::UnsupportedMotion);
  EXPECT_EQ(unsupported.current.result.work,0u);
  EXPECT_FALSE(native::NativeStorageDomain::FromPaths(paths[0],paths[0],53).eligible());
}
} // namespace zero_storage
} // namespace represented_interval_test
