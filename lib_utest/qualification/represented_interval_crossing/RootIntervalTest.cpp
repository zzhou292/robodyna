// SPDX-License-Identifier: MIT
#include "Oracle.h"
#include "ResultAssertions.h"
#include "lib_src/collision/represented_interval_crossing/RootIntervalQualification.h"
#include <cerrno>
#include <cfenv>
#if defined(__x86_64__) && defined(__SSE2__)
#include <xmmintrin.h>
#endif

namespace represented_interval_test {
namespace root_filter=ct::represented_interval_crossing;
using RootPaths=std::array<ct::RepresentedTrianglePath,2>;

root_filter::RootIntervalComparison CheckRootFilter(const RootPaths& paths,
                                                   ct::RepresentedIntervalLimits limits={}) {
  const auto result=root_filter::CompareRootIntervalFilter(paths[0],paths[1],limits);
  EXPECT_EQ(result.status,S::Ok);
  if (result.filter.separated) {
    EXPECT_TRUE(result.domain.eligible);
    EXPECT_TRUE(ExactRootIntervalCertificate(result.input.vertices,result.filter.axis));
    EXPECT_EQ(result.current.classification,C::CertifiedSeparated);
    EXPECT_EQ(result.current.reason,R::None);
    EXPECT_EQ(result.current.work,1u);
    EXPECT_EQ(result.current_exact_cells,0u);
    EXPECT_EQ(result.counters.attempts,1u);
  } else {
    SameResult(result.current,result.original);
    EXPECT_EQ(result.current_exact_cells,result.original_exact_cells);
  }
  return result;
}

TEST(RepresentedRootIntervalFilter, StaticSeparationSkipsAllExactSamplesWithIndependentProof) {
  const RootPaths paths{Static(10,BaseTriangle()),Static(20,BaseTriangle(1))};
  const auto result=CheckRootFilter(paths);
  ASSERT_TRUE(result.filter.separated);
  EXPECT_EQ(result.original_exact_cells,1u);
  SameResult(result.current,result.original);
  auto owner=Owner();SameResult(One(owner,{paths[0],paths[1]}),result.current);
  auto broken=result.input;broken.vertices[1][0][0].z=0;broken.vertices[1][1][0].z=0;
  EXPECT_FALSE(ExactRootIntervalCertificate(broken.vertices,result.filter.axis));
}

TEST(RepresentedRootIntervalFilter, RelativeDriftAndObliqueAxesReuseTheSameExactOracle) {
  const std::array<ct::Vec3,3> oblique{{{0,0,0},{2,0,2},{0,2,2}}};
  auto b=oblique,next=b;
  for(unsigned i=0;i<3;++i){b[i].z+=.125;next[i].z+=.25;}
  ASSERT_TRUE(CheckRootFilter({Static(10,oblique),Path(20,b,next)}).filter.separated);
  const std::array<ct::Vec3,3> plane{{{0,0,0},{0,1,0},{0,0,1}}};
  auto a_next=plane,b_base=plane,b_next=plane;
  for(unsigned i=0;i<3;++i){a_next[i].x=1;b_base[i].x=.125;b_next[i].x=1.126;}
  ASSERT_TRUE(CheckRootFilter({Path(10,plane,a_next),Path(20,b_base,b_next)}).filter.separated);
}

TEST(RepresentedRootIntervalFilter, AuthenticatedSharedVertexBypassesBoundsWithoutGrantingContactAuthority) {
  auto a=Static(10,BaseTriangle());
  auto b=Static(20,{{{0,0,0},{-2,0,0},{0,-2,0}}});
  b.vertices[0].key=a.vertices[0].key;
  for(unsigned edge=0;edge<3;++edge)
    b.edge_keys[edge]=Edge(b.vertices[edge].key,b.vertices[(edge+1)%3].key);
  const auto result=CheckRootFilter({a,b});
  EXPECT_FALSE(result.filter.separated);
  EXPECT_EQ(result.counters.shared_vertex_bypasses,1u);
  EXPECT_EQ(result.counters.attempts,0u);
  EXPECT_GT(result.current_exact_cells,0u);
  SameResult(result.current,result.original);
  auto owner=Owner();SameResult(One(owner,{a,b}),result.original);
  const auto before=owner.results();
  // Shared identity is never permission to ignore a contradictory trajectory.
  b.vertices[0].endpoint[1].z=1;
  const auto malformed=root_filter::CompareRootIntervalFilter(a,b,{});
  EXPECT_EQ(malformed.status,S::IdentityMismatch);
  EXPECT_EQ(malformed.counters.shared_vertex_bypasses,0u);
  const ct::RepresentedTrianglePath paths[]{a,b};
  const ct::RepresentedTrianglePair pair{0,1};
  EXPECT_EQ(owner.Certify(paths,2,&pair,1).status,S::IdentityMismatch);
  ASSERT_EQ(owner.results().data,before.data);ASSERT_EQ(owner.results().count,1u);
  SameResult(owner.results().data[0],result.original);
}

TEST(RepresentedRootIntervalFilter, StrictUlpGapsAndTouchingKeepTheirDistinctOutcomes) {
  for(int sign:{-1,0,1}) {
    const double height=sign?std::nextafter(1.,sign>0?INFINITY:-INFINITY):1.;
    const auto result=CheckRootFilter({Static(10,BaseTriangle(1)),Static(20,BaseTriangle(height))});
    if(sign) EXPECT_TRUE(result.filter.separated);
    else {EXPECT_FALSE(result.filter.separated);EXPECT_EQ(result.current.classification,C::CertifiedCrossingContact);}
  }
}

TEST(RepresentedRootIntervalFilter, InteriorContactAndDegeneracyFallBackWithoutRelabeling) {
  ct::RepresentedIntervalLimits limits;limits.max_work_per_pair=3;limits.max_depth=8;
  const RootPaths crossing{Static(10,BaseTriangle()),Path(20,BaseTriangle(1),BaseTriangle(-2))};
  const auto collision=CheckRootFilter(crossing,limits);
  EXPECT_FALSE(collision.filter.separated);EXPECT_EQ(collision.current.classification,C::Unresolved);
  EXPECT_TRUE(ExactOracleAt(crossing[0],crossing[1],1,3).intersects);
  auto next=BaseTriangle();next[1].x=-4;
  EXPECT_FALSE(CheckRootFilter({Path(10,BaseTriangle(),next),Static(20,BaseTriangle(2))},limits).filter.separated);
  const std::array<ct::Vec3,3> collapsed{{{0,0,0},{0,0,0},{0,0,0}}};
  const auto degenerate=CheckRootFilter({Path(10,collapsed,BaseTriangle()),Static(20,BaseTriangle())},limits);
  EXPECT_FALSE(degenerate.filter.separated);
  EXPECT_EQ(degenerate.current.classification,C::CertifiedCrossingContact);
  EXPECT_EQ(degenerate.current.witness_time_depth,1u);
}

TEST(RepresentedRootIntervalFilter, CanonicalKeysFixVertexWindingAndCallerOrder) {
  const std::array<std::array<unsigned,3>,6> orders{{{{0,1,2}},{{0,2,1}},{{1,0,2}},{{1,2,0}},{{2,0,1}},{{2,1,0}}}};
  const RootPaths paths{Static(10,BaseTriangle()),Static(20,BaseTriangle(1))};
  const auto reference=CheckRootFilter(paths);
  for(const auto& a:orders)for(const auto& b:orders) {
    const auto result=CheckRootFilter({Permute(paths[1],b),Permute(paths[0],a)});
    EXPECT_EQ(result.filter.separated,reference.filter.separated);
    EXPECT_EQ(result.filter.axis.x,reference.filter.axis.x);
    EXPECT_EQ(result.filter.axis.y,reference.filter.axis.y);
    EXPECT_EQ(result.filter.axis.z,reference.filter.axis.z);
    SameResult(result.current,reference.current);
  }
}

TEST(RepresentedRootIntervalFilter, ArithmeticAmbiguityAndWideDomainRetainExactFallback) {
  for(double scale:{std::ldexp(1.,-1000),std::ldexp(1.,151),std::ldexp(1.,900)}) {
    auto a=BaseTriangle(),b=BaseTriangle(1);
    for(auto* points:{&a,&b})for(auto& p:*points){p.x*=scale;p.y*=scale;p.z*=scale;}
    const auto result=CheckRootFilter({Static(10,a),Static(20,b)});
    EXPECT_FALSE(result.filter.separated);
    SameResult(result.current,result.original);
  }
  auto invalid=Static(20,BaseTriangle());invalid.vertices[0].endpoint[0].x=NAN;
  EXPECT_EQ(root_filter::CompareRootIntervalFilter(Static(10,BaseTriangle()),invalid,{}).status,S::InvalidInput);
  auto unsupported=Static(20,BaseTriangle(1));unsupported.motion=ct::RepresentedMotion::RigidArc;
  const auto arc=CheckRootFilter({Static(10,BaseTriangle()),unsupported});
  EXPECT_EQ(arc.current.reason,R::UnsupportedMotion);EXPECT_FALSE(arc.filter.separated);
}

TEST(RepresentedRootIntervalFilter, FloatingEnvironmentFallsBackAndRestoresFlagsAndErrno) {
  const RootPaths paths{Static(10,BaseTriangle()),Static(20,BaseTriangle(1))};
  struct Restore {
    std::fenv_t saved;
    int old_errno=errno;
    Restore(){std::fegetenv(&saved);}
    ~Restore(){std::fesetenv(&saved);errno=old_errno;}
  } restore;
  for(int mode:{FE_DOWNWARD,FE_UPWARD,FE_TOWARDZERO}) {
    ASSERT_EQ(std::fesetround(mode),0);
    const auto result=CheckRootFilter(paths);
    EXPECT_FALSE(result.filter.separated);
    EXPECT_EQ(std::fegetround(),mode);
  }
  ASSERT_EQ(std::fesetround(FE_TONEAREST),0);
#if defined(__x86_64__) && defined(__SSE2__)
  const auto csr=_mm_getcsr();
  for(unsigned flush:{0x8000u,0x40u,0x8040u}) {
    _mm_setcsr(csr|flush);
    const auto result=CheckRootFilter(paths);
    EXPECT_FALSE(result.filter.separated);EXPECT_EQ(_mm_getcsr(),csr|flush);
  }
  _mm_setcsr(csr);
  auto underflow_input=root_filter::CompareRootIntervalFilter(paths[0],paths[1],{}).input;
  const double tiny=std::ldexp(1.,-1000);
  for(auto& side:underflow_input.vertices)for(auto& endpoint:side)for(auto& p:endpoint) {
    p.x*=tiny;p.y*=tiny;p.z*=tiny;
  }
  // Added floating arithmetic must not introduce a trap that exact predicates
  // would avoid. Clear pending flags, unmask under/overflow, and require the
  // inconclusive underflow path to restore those masks without raising flags.
  const auto traps=(csr&~0x3fu)&~0x0c00u;
  _mm_setcsr(traps);
  EXPECT_FALSE(root_filter::ProveRootIntervalSeparation(underflow_input).separated);
  EXPECT_EQ(_mm_getcsr(),traps);
  _mm_setcsr(csr);
#endif
  std::feclearexcept(FE_ALL_EXCEPT);std::feraiseexcept(FE_DIVBYZERO);
  const int flags=std::fetestexcept(FE_ALL_EXCEPT);
  errno=EDOM;
  const auto admitted=root_filter::CompareRootIntervalFilter(paths[0],paths[1],{});
  EXPECT_TRUE(admitted.filter.separated);
  EXPECT_EQ(std::fetestexcept(FE_ALL_EXCEPT),flags);EXPECT_EQ(errno,EDOM);
}

TEST(RepresentedRootIntervalFilter, IndependentDyadicFamiliesNeverAdmitAnUnprovedGap) {
  for(int x:{-2,0,2})for(int y:{-2,0,2})for(int z:{-1,0,1}) {
    auto first=BaseTriangle(),second=first,next=first;
    for(unsigned i=0;i<3;++i){second[i].x+=x;second[i].y+=y;second[i].z+=z;next[i]=second[i];next[i].x+=.25;}
    ct::RepresentedIntervalLimits limits;limits.max_work_per_pair=7;limits.max_depth=3;
    CheckRootFilter({Static(10,first),Path(20,second,next)},limits);
  }
}

TEST(RepresentedRootIntervalFilter, WorkersFailurePublicationAndRetryRetainOwnerProtocol) {
  const std::vector<ct::RepresentedTrianglePath> paths{Static(10,BaseTriangle()),Static(20,BaseTriangle(1)),Static(30,BaseTriangle(2))};
  const ct::RepresentedTrianglePair pairs[]{{0,1},{0,2}};
  for(unsigned workers:{1u,4u}) {
    ct::RepresentedIntervalLimits limits;limits.worker_count=workers;limits.max_total_work=1;
    auto owner=Owner(limits);
    const auto prior=One(owner,{paths[0],paths[1]});const auto view=owner.results();
    const auto failed=owner.Certify(paths.data(),paths.size(),pairs,2);
    EXPECT_EQ(failed.status,S::ResourceLimit);EXPECT_EQ(failed.work,1u);EXPECT_EQ(failed.rejected_pair_work,1u);
    ASSERT_EQ(owner.results().data,view.data);ASSERT_EQ(owner.results().count,1u);SameResult(owner.results().data[0],prior);
    const auto retry=One(owner,{paths[0],paths[2]});
    SameResult(retry,CheckRootFilter({paths[0],paths[2]},limits).current);
  }
}
} // namespace represented_interval_test
