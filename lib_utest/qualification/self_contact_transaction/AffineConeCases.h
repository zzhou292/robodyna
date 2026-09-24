// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

// Included after the owning native triangle, rigid-path and result helpers.
struct AffineConeGeometry {
  c::CurrentFixedTriangle first=Triangle(10,{1,2,3},
      {{{0,0,0},{1.03125,1.984375,0},{-1.96875,-4.015625,0}}});
  c::CurrentFixedTriangle second=Triangle(20,{1,4,5},
      {{{0,0,0},{.96875,2.015625,0},{-3.03125,-5.984375,0}}});
};

cone_direction_test::Rays ExactRays(const c::Vec3 (&rays)[8]) {
  cone_direction_test::Rays result;
  for(unsigned i=0;i<8;++i)result[i]=cone_direction_test::Exact(rays[i]);
  return result;
}
bool StreamedStrictDirection(const c::Vec3 (&rays)[8],unsigned* visited) {
  const auto exact=ExactRays(rays);
  sct::ConeDirections stream(rays);
  c::Vec3 direction;
  while(stream.Next(&direction)) {
    if(!c::IsFinite(direction))continue;
    if(cone_direction_test::Strict(exact,cone_direction_test::Exact(direction))) {
      *visited=stream.emitted();return true;
    }
  }
  *visited=stream.emitted();return false;
}

TEST(SelfContactAffineCone, IteratorStreamsExactlyAllNinetyTwoSubsetsInOrder) {
  const c::Vec3 rays[8]{{1,2,3},{2,-1,4},{-1,4,2},{3,2,-1},{1,-3,2},{4,1,1},{2,3,4},{-2,3,1}};
  sct::ConeDirections stream(rays);c::Vec3 actual;
  EXPECT_FALSE(stream.Next(nullptr));EXPECT_EQ(stream.emitted(),0u);
  const auto same=[&](c::Vec3 expected) {
    ASSERT_TRUE(stream.Next(&actual));EXPECT_EQ(actual.x,expected.x);
    EXPECT_EQ(actual.y,expected.y);EXPECT_EQ(actual.z,expected.z);
  };
  for(const auto ray:rays)same(ray);
  for(unsigned i=0;i<8;++i)for(unsigned j=i+1;j<8;++j) {
    const auto edge=c::Subtract(rays[j],rays[i]);
    same(c::geometry_detail::Cross(edge,c::geometry_detail::Cross(rays[i],edge)));
  }
  for(unsigned i=0;i<8;++i)for(unsigned j=i+1;j<8;++j)for(unsigned k=j+1;k<8;++k)
    same(c::geometry_detail::Cross(c::Subtract(rays[j],rays[i]),c::Subtract(rays[k],rays[i])));
  EXPECT_EQ(stream.emitted(),sct::ConeDirections::MaximumDirections);
  EXPECT_FALSE(stream.Next(&actual));EXPECT_FALSE(stream.Next(&actual));
}

TEST(SelfContactAffineCone, ExactFeasibilityHandlesDuplicateRankDeficientTouchingAndOverlappingRays) {
  const c::Vec3 sets[][8]{
    {{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0}},
    {{1,2,0},{1,-2,0},{1,1,0},{1,-1,0},{1,2,0},{1,-2,0},{1,1,0},{1,-1,0}},
    {{1,0,0},{-1,0,0},{1,0,0},{-1,0,0},{1,0,0},{-1,0,0},{1,0,0},{-1,0,0}},
    {{1,0,0},{0,1,0},{0,0,1},{-1,0,0},{0,-1,0},{0,0,-1},{1,1,1},{-1,-1,-1}},
    {{0,0,0},{1,1,1},{1,1,1},{1,1,1},{1,1,1},{1,1,1},{1,1,1},{1,1,1}}};
  for(unsigned row=0;row<5;++row) {
    SCOPED_TRACE(row);unsigned visited=0;
    const bool expected=row<2;
    EXPECT_EQ(cone_direction_test::Feasible(ExactRays(sets[row])),expected);
    EXPECT_EQ(StreamedStrictDirection(sets[row],&visited),expected);
    EXPECT_LE(visited,92u);if(!expected)EXPECT_EQ(visited,92u);
  }
}

TEST(SelfContactAffineCone, GenericObliqueCoplanarFamilyPreservesFeasibilityAcrossScalingAndRotations) {
  const AffineConeGeometry geometry;
  const c::Vec3 base[8]{geometry.first.vertices[1],geometry.first.vertices[2],
    c::Scale(geometry.second.vertices[1],-1),c::Scale(geometry.second.vertices[2],-1),
    geometry.first.vertices[1],geometry.first.vertices[2],
    c::Scale(geometry.second.vertices[1],-1),c::Scale(geometry.second.vertices[2],-1)};
  for(int exponent:{-200,-20,0,20,200})for(const auto& axes:ConePermutations) {
    c::Vec3 rays[8];
    for(unsigned i=0;i<8;++i) {
      const double v[]{base[i].x,base[i].y,base[i].z};
      rays[i]={std::ldexp(v[axes[0]],exponent),std::ldexp(v[axes[1]],exponent),std::ldexp(v[axes[2]],exponent)};
    }
    EXPECT_TRUE(cone_direction_test::Feasible(ExactRays(rays)));
    unsigned visited=0;EXPECT_TRUE(StreamedStrictDirection(rays,&visited));EXPECT_LE(visited,92u);
  }
}

TEST(SelfContactAffineCone, NewProofResolvesLocalGeometryAfterBothOriginalAlternativesFail) {
  const AffineConeGeometry geometry;
  auto first_next=geometry.first,second_next=geometry.second;
  for(auto* triangle:{&first_next,&second_next})for(auto& vertex:triangle->vertices)vertex=c::Scale(vertex,1.25);
  const auto compared=sct::CompareAffineConeSearch(geometry.first,first_next,ZeroQuadratic(),
      geometry.second,second_next,ZeroQuadratic(),1,255,8);
  EXPECT_NE(compared.original.report.status,sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
  EXPECT_EQ(compared.current.report.status,sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
  EXPECT_EQ(compared.current.report.work,1u);
  EXPECT_EQ(compared.original.counters.affine_searches,0u);
  EXPECT_EQ(compared.current.counters.affine_searches,1u);
  EXPECT_GT(compared.current.counters.affine_directions,0u);
  EXPECT_LE(compared.current.counters.affine_directions,92u);
  EXPECT_FALSE(compared.current.counters.saturated);
  const auto actual=sct::CertifyQuadraticLocalTopology(geometry.first,first_next,ZeroQuadratic(),
      geometry.second,second_next,ZeroQuadratic(),1,255,8);
  sct::test::ExpectResult(actual,compared.current.report);
  const auto contact=sct::CertifyQuadraticLocalContact(geometry.first,first_next,ZeroQuadratic(),.001,
      geometry.second,second_next,ZeroQuadratic(),.001,1,255,8);
  // The large dilation is topologically local, but the independent conservative
  // residual/thickness screen cannot certify its unmasked features. A geometry
  // certificate must never override that separate physical contact obligation.
  EXPECT_EQ(contact.status,sct::NonlinearSeparationStatus::PotentialContact);
  const auto policy=sct::CertifyQuadraticFacetPolicyCoverage(geometry.first,first_next,ZeroQuadratic(),.001,
      geometry.second,second_next,ZeroQuadratic(),.001,1,nullptr,0,nullptr,0,255,8);
  EXPECT_EQ(policy.status,sct::NonlinearSeparationStatus::MissingAcceptedOwner);
  // A known rational halfspace is an independent oracle for the fixture only.
  const c::Vec3 axis{2,-1,0};
  for(const auto* triangles:std::array<const c::CurrentFixedTriangle*,2>{&geometry.first,&first_next})for(unsigned v=1;v<3;++v)
    EXPECT_GT(cone_direction_test::ArmDot(triangles->vertices[v],triangles->vertices[0],axis),0);
  for(const auto* triangles:std::array<const c::CurrentFixedTriangle*,2>{&geometry.second,&second_next})for(unsigned v=1;v<3;++v)
    EXPECT_LT(cone_direction_test::ArmDot(triangles->vertices[v],triangles->vertices[0],axis),0);
}

TEST(SelfContactAffineCone, StaticGeometryPassesBothTopologyAndPositiveThicknessObligations) {
  const AffineConeGeometry geometry;
  // Static endpoints have no residual motion bound. Keep the same source
  // geometry and positive shell half-thickness as the large-motion negative.
  const auto contact = sct::CertifyQuadraticLocalContact(
      geometry.first, geometry.first, ZeroQuadratic(), .001,
      geometry.second, geometry.second, ZeroQuadratic(), .001, 1, 255, 8);
  ASSERT_EQ(contact.status, sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
  EXPECT_EQ(contact.work, 1u);
  const auto policy = sct::CertifyQuadraticFacetPolicyCoverage(
      geometry.first, geometry.first, ZeroQuadratic(), .001,
      geometry.second, geometry.second, ZeroQuadratic(), .001, 1,
      nullptr, 0, nullptr, 0, 255, 8);
  sct::test::ExpectResult(policy, contact);
}

TEST(SelfContactAffineCone, ExtremeRoundedDirectionsRemainBoundedAndInconclusive) {
  const AffineConeGeometry geometry;
  for(int exponent:{-512,512}) {
    c::Vec3 rays[8];unsigned count=0;
    for(unsigned endpoint=0;endpoint<2;++endpoint)
      for(const auto* triangle:{&geometry.first,&geometry.second})
        for(unsigned vertex=1;vertex<3;++vertex) {
          const auto value=triangle->vertices[vertex];
          const double sign=triangle==&geometry.first?1:-1;
          rays[count++]={std::ldexp(sign*value.x,exponent),std::ldexp(sign*value.y,exponent),0};
        }
    ASSERT_TRUE(cone_direction_test::Feasible(ExactRays(rays)));
    unsigned visited=0;
    EXPECT_FALSE(StreamedStrictDirection(rays,&visited));
    EXPECT_EQ(visited,92u);
  }
}

TEST(SelfContactAffineCone, InfeasibleRootSearchExhaustsOnceAndDoesNotRepeatInChildren) {
  const AffineConeGeometry geometry;
  auto next_first=geometry.first,next_second=geometry.second;
  // A(t)=[[1,-t],[t,1]] has positive determinant 1+t*t, so endpoints and
  // interior retain shared-vertex-only topology, but no single root axis
  // separates these narrow dual cones across their 45-degree rotation.
  for(auto* triangle:{&next_first,&next_second})for(auto& vertex:triangle->vertices)
    vertex={vertex.x-vertex.y,vertex.x+vertex.y,vertex.z};
  c::Vec3 rays[8];unsigned count=0;
  for(const auto& endpoints:std::array<std::array<const c::CurrentFixedTriangle*,2>,2>{{
      {{&geometry.first,&geometry.second}},{{&next_first,&next_second}}}})
    for(unsigned side=0;side<2;++side)for(unsigned vertex=1;vertex<3;++vertex)
      rays[count++]=c::Scale(endpoints[side]->vertices[vertex],side?-1:1);
  ASSERT_FALSE(cone_direction_test::Feasible(ExactRays(rays)));
  const auto compared=sct::CompareAffineConeSearch(geometry.first,next_first,ZeroQuadratic(),
      geometry.second,next_second,ZeroQuadratic(),1,255,8);
  sct::test::ExpectResult(compared.current.report,compared.original.report);
  EXPECT_EQ(compared.current.counters.affine_searches,1u);
  EXPECT_EQ(compared.current.counters.affine_directions,92u);
  EXPECT_GT(compared.current.counters.cells,1u);
}

TEST(SelfContactAffineCone, ExistingProofsRetainTheirOriginalReportsAndDoNotSearchAgain) {
  const CapturedConeGeometry geometry;
  const auto compared=sct::CompareAffineConeSearch(geometry.first,geometry.first_prepared,ZeroQuadratic(),
      geometry.second,geometry.second_prepared,ZeroQuadratic(),2e-7,1,0);
  sct::test::ExpectResult(compared.current.report,compared.original.report);
  EXPECT_EQ(compared.current.counters.affine_searches,0u);
  EXPECT_EQ(compared.current.counters.affine_directions,0u);
}

TEST(SelfContactAffineCone, NoSearchInGeneralLedgerRecursiveCellsOrCurvedGeometry) {
  const AffineConeGeometry geometry;
  for(unsigned depth:{0u,3u,8u}) {
    const auto ledger=sct::CompareSharedVertexCoverageOrders(geometry.first,geometry.first,ZeroQuadratic(),.001,
        geometry.second,geometry.second,ZeroQuadratic(),.001,1,nullptr,0,255,depth);
    EXPECT_EQ(ledger.cone_first.counters.affine_searches,0u);
    EXPECT_EQ(ledger.polynomial_first.counters.affine_searches,0u);
    EXPECT_NE(ledger.cone_first.report.status,sct::NonlinearSeparationStatus::CertifiedAcceptedCoverage);
  }
  CurvedSharedVertex curved;curved.DeriveActualRigidCoefficient();
  const auto compared=sct::CompareAffineConeSearch(curved.first,curved.first,ZeroQuadratic(),
      curved.second,curved.second,curved.curved,Duration,255,12);
  sct::test::ExpectResult(compared.current.report,compared.original.report);
  EXPECT_EQ(compared.current.counters.affine_searches,0u);
  EXPECT_NE(compared.current.report.status,sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
  const auto invalid=sct::CompareAffineConeSearch(geometry.first,geometry.first,ZeroQuadratic(),
      geometry.second,geometry.second,ZeroQuadratic(),1,0,0);
  EXPECT_EQ(invalid.current.counters.affine_searches,0u);
}

TEST(SelfContactAffineCone, EndpointNonlocalIntersectionAndMismatchedSourcePathsCannotGainAdmission) {
  const AffineConeGeometry geometry;
  for(unsigned mutation=0;mutation<3;++mutation) {
    auto b=geometry.second,next=b;
    if(mutation==0){b.vertices[1]=geometry.first.vertices[1];next=b;}
    if(mutation==1)next.vertices[0].x+=.01;
    if(mutation==2)next.vertex_keys[0].first+=100;
    const auto result=sct::CompareAffineConeSearch(geometry.first,geometry.first,ZeroQuadratic(),b,next,ZeroQuadratic(),1,255,8);
    sct::test::ExpectResult(result.current.report,result.original.report);
    EXPECT_EQ(result.current.counters.affine_searches,0u);
    EXPECT_NE(result.current.report.status,sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
  }
}
