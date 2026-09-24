// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Arithmetic.h"
namespace tlfea::contact::represented_interval_crossing::native {
template <unsigned Bits, class IntegerPolicy = BoostIntegerPolicy<Bits>>
struct Geometry : Arithmetic<Bits, IntegerPolicy> {
  using Base = Arithmetic<Bits, IntegerPolicy>;
  TL_MATH_HOST_DEVICE explicit Geometry(ArithmeticContext& context) noexcept : Base(context) {}
  using typename Base::Dyadic;
  using typename Base::ExactVec3;
  using typename Base::ExactTriangle;
  using Base::Add;
  using Base::Subtract;
  using Base::Cross;
  using Base::Dot;
  using Base::Zero;
  using Base::Sign;
  using Base::Compare;
  using Base::Component;
  using Base::Scale;
  // Keep key ordering distinct from inherited dyadic numeric comparison.
  TL_MATH_HOST_DEVICE static int Compare(const FacetEdgeKey& a, const FacetEdgeKey& b) noexcept {
    return native::Compare(a, b);
  }
  TL_MATH_HOST_DEVICE ExactVec3 Edge(const ExactTriangle& triangle, unsigned edge) {
    return Subtract(triangle.vertex[(edge + 1) % 3],
                    triangle.vertex[edge]);
  }

  TL_MATH_HOST_DEVICE ExactVec3 Normal(const ExactTriangle& triangle, NormalCounters* counters = nullptr) {
    CountNormalOperation(counters, &NormalCounters::normal_evaluations);
    return Cross(Edge(triangle, 0),
                 Subtract(triangle.vertex[2], triangle.vertex[0]));
  }

  TL_MATH_HOST_DEVICE bool Degenerate(const ExactTriangle& triangle, NormalCounters* counters = nullptr) {
    return Zero(Normal(triangle, counters));
  }

  TL_MATH_HOST_DEVICE bool SeparatedOnAxis(const ExactTriangle& a, const ExactTriangle& b,
                       const ExactVec3& axis) {
    if (Zero(axis))
      return false;
    Dyadic minimum_a = Dot(a.vertex[0], axis);
    Dyadic maximum_a = minimum_a;
    Dyadic minimum_b = Dot(b.vertex[0], axis);
    Dyadic maximum_b = minimum_b;
    for (unsigned i = 1; i < 3; ++i) {
      const Dyadic pa = Dot(a.vertex[i], axis);
      const Dyadic pb = Dot(b.vertex[i], axis);
      if (Compare(pa, minimum_a) < 0)
        minimum_a = pa;
      if (Compare(pa, maximum_a) > 0)
        maximum_a = pa;
      if (Compare(pb, minimum_b) < 0)
        minimum_b = pb;
      if (Compare(pb, maximum_b) > 0)
        maximum_b = pb;
    }
    return Compare(maximum_a, minimum_b) < 0 ||
           Compare(maximum_b, minimum_a) < 0;
  }

  struct StaticIntersection {
    bool intersects = false;
    bool coplanar = false;
  };

  TL_MATH_HOST_DEVICE StaticIntersection Intersects(const ExactTriangle& a,
                                const ExactTriangle& b,
                                const ExactVec3& normal_a,
                                const ExactVec3& normal_b,
                                bool common_endpoint = false,
                                CommonPointCounters* common_point_counters = nullptr) {
    StaticIntersection result;
    result.coplanar = true;
    for (unsigned i = 0; i < 3; ++i) {
      result.coplanar =
          result.coplanar &&
          Sign(Dot(Subtract(b.vertex[i], a.vertex[0]), normal_a)) == 0 &&
          Sign(Dot(Subtract(a.vertex[i], b.vertex[0]), normal_b)) == 0;
    }
    // The exact coplanarity classification above remains unchanged. A point
    // contained in both closed triangles makes strict projection separation
    // impossible on every SAT axis. The caller admits this only after original
    // nondegeneracy checks and the existing no-allocation arithmetic-domain proof.
    if (common_endpoint) {
      CountCommonPointOperation(common_point_counters, &CommonPointCounters::static_sat_bypasses);
      result.intersects = true;
      return result;
    }
    if (SeparatedOnAxis(a, b, normal_a) ||
        SeparatedOnAxis(a, b, normal_b))
      return result;
    for (unsigned i = 0; i < 3; ++i)
      for (unsigned j = 0; j < 3; ++j)
        if (SeparatedOnAxis(a, b, Cross(Edge(a, i), Edge(b, j))))
          return result;
    if (result.coplanar) {
      for (unsigned i = 0; i < 3; ++i) {
        if (SeparatedOnAxis(a, b, Cross(normal_a, Edge(a, i))) ||
            SeparatedOnAxis(a, b, Cross(normal_a, Edge(b, i))))
          return result;
      }
    }
    result.intersects = true;
    return result;
  }

  TL_MATH_HOST_DEVICE StaticIntersection Intersects(const ExactTriangle& a,
                                const ExactTriangle& b,
                                NormalCounters* counters = nullptr) {
    // Retained uncached oracle: these exact normal evaluations retain their
    // original sequence and checked-arithmetic failure boundary.
    const ExactVec3 normal_a = Normal(a, counters);
    const ExactVec3 normal_b = Normal(b, counters);
    return Intersects(a, b, normal_a, normal_b);
  }

  TL_MATH_HOST_DEVICE bool PointInClosedTriangle(const ExactVec3& point,
                             const ExactTriangle& triangle,
                             const ExactVec3& normal) {
    if (Sign(Dot(Subtract(point, triangle.vertex[0]), normal)) != 0)
      return false;
    int orientation = 0;
    for (unsigned i = 0; i < 3; ++i) {
      const int sign =
          Sign(Dot(Cross(Edge(triangle, i),
                         Subtract(point, triangle.vertex[i])),
                   normal));
      if (sign) {
        if (orientation && sign != orientation)
          return false;
        orientation = sign;
      }
    }
    return true;
  }

  TL_MATH_HOST_DEVICE bool PointInClosedTriangle(const ExactVec3& point,
                             const ExactTriangle& triangle,
                             NormalCounters* counters = nullptr) {
    const ExactVec3 normal = Normal(triangle, counters);
    return PointInClosedTriangle(point, triangle, normal);
  }

  TL_MATH_HOST_DEVICE bool SegmentsIntersect(const ExactVec3& a0, const ExactVec3& a1,
                         const ExactVec3& b0, const ExactVec3& b1) {
    const ExactVec3 a = Subtract(a1, a0);
    const ExactVec3 b = Subtract(b1, b0);
    const ExactVec3 delta = Subtract(b0, a0);
    const ExactVec3 normal = Cross(a, b);
    if (!Zero(normal)) {
      if (Sign(Dot(delta, normal)) != 0)
        return false;
      const Dyadic denominator = Dot(normal, normal);
      const Dyadic parameter_a = Dot(Cross(delta, b), normal);
      const Dyadic parameter_b = Dot(Cross(delta, a), normal);
      return Sign(parameter_a) >= 0 &&
             Compare(parameter_a, denominator) <= 0 &&
             Sign(parameter_b) >= 0 &&
             Compare(parameter_b, denominator) <= 0;
    }
    if (!Zero(Cross(a, delta)))
      return false;
    unsigned component = 0;
    for (unsigned i = 1; i < 3; ++i)
      if (Sign(Component(a, component)) == 0)
        component = i;
    Dyadic aa0 = Component(a0, component);
    Dyadic aa1 = Component(a1, component);
    Dyadic bb0 = Component(b0, component);
    Dyadic bb1 = Component(b1, component);
    if (Compare(aa1, aa0) < 0)
      portable::swap(aa0, aa1);
    if (Compare(bb1, bb0) < 0)
      portable::swap(bb0, bb1);
    return Compare(aa1, bb0) >= 0 && Compare(bb1, aa0) >= 0;
  }

  TL_MATH_HOST_DEVICE RepresentedFeaturePathKey IntersectionFeature(
      const RepresentedTrianglePath& path_a,
      const RepresentedTrianglePath& path_b, const ExactTriangle& a,
      const ExactTriangle& b,
      const ExactVec3* normal_a = nullptr, const ExactVec3* normal_b = nullptr,
      NormalCounters* counters = nullptr, bool skip_dominated_edges = false,
      ExactPathCounters* path_counters = nullptr) {
    bool have = false;
    RepresentedFeaturePathKey result;
    for (unsigned vertex = 0; vertex < 3; ++vertex) {
      CountExactPathOperation(path_counters, &ExactPathCounters::vertex_face_tests);
      if (normal_b ? PointInClosedTriangle(a.vertex[vertex], b, *normal_b)
                   : PointInClosedTriangle(a.vertex[vertex], b, counters)) {
        RepresentedFeaturePathKey candidate;
        candidate.kind = RepresentedFeatureKind::VertexFace;
        candidate.vertex = path_a.vertices[vertex].key;
        candidate.face = path_b.key;
        ConsiderFeature(candidate, &have, &result);
      }
      CountExactPathOperation(path_counters, &ExactPathCounters::vertex_face_tests);
      if (normal_a ? PointInClosedTriangle(b.vertex[vertex], a, *normal_a)
                   : PointInClosedTriangle(b.vertex[vertex], a, counters)) {
        RepresentedFeaturePathKey candidate;
        candidate.kind = RepresentedFeatureKind::VertexFace;
        candidate.vertex = path_b.vertices[vertex].key;
        candidate.face = path_a.key;
        ConsiderFeature(candidate, &have, &result);
      }
    }
    // All six directed VF predicates have contributed to the canonical minimum.
    // Every EE kind sorts after VF, so none can replace this witness. The caller
    // admits skipping only within the independently proved nonallocating domain.
    static_assert(RepresentedFeatureKind::VertexFace < RepresentedFeatureKind::EdgeEdge);
    if (skip_dominated_edges && have && result.kind == RepresentedFeatureKind::VertexFace) {
      CountExactPathOperation(path_counters, &ExactPathCounters::dominated_edge_loops);
      return result;
    }
    for (unsigned edge_a = 0; edge_a < 3; ++edge_a) {
      for (unsigned edge_b = 0; edge_b < 3; ++edge_b) {
        CountExactPathOperation(path_counters, &ExactPathCounters::edge_edge_tests);
        if (!SegmentsIntersect(a.vertex[edge_a],
                               a.vertex[(edge_a + 1) % 3],
                               b.vertex[edge_b],
                               b.vertex[(edge_b + 1) % 3]))
          continue;
        RepresentedFeaturePathKey candidate;
        candidate.kind = RepresentedFeatureKind::EdgeEdge;
        if (Compare(path_a.edge_keys[edge_a],
                    path_b.edge_keys[edge_b]) <= 0) {
          candidate.edges[0] = path_a.edge_keys[edge_a];
          candidate.edges[1] = path_b.edge_keys[edge_b];
        } else {
          candidate.edges[0] = path_b.edge_keys[edge_b];
          candidate.edges[1] = path_a.edge_keys[edge_a];
        }
        ConsiderFeature(candidate, &have, &result);
      }
    }
    if (!have)
      result.kind = RepresentedFeatureKind::TriangleIntersection;
    return result;
  }

  TL_MATH_HOST_DEVICE bool RegularCell(const ExactVec3 normal[3]) {
    for (unsigned component = 0; component < 3; ++component) {
      const Dyadic first = Component(normal[0], component);
      const Dyadic middle = Component(normal[1], component);
      const Dyadic last = Component(normal[2], component);
      // Twice the middle Bernstein coefficient has the sign of
      // 4*n(mid)-n(lower)-n(upper).
      const Dyadic control =
          Subtract(Subtract(Scale(middle, 4), first), last);
      const int a = Sign(first);
      const int b = Sign(control);
      const int c = Sign(last);
      if (a && a == b && b == c)
        return true;
    }
    return false;
  }

  TL_MATH_HOST_DEVICE bool RegularCell(const ExactTriangle samples[3], NormalCounters* counters = nullptr) {
    ExactVec3 normal[3] = {Normal(samples[0], counters), Normal(samples[1], counters),
                           Normal(samples[2], counters)};
    return RegularCell(normal);
  }

  TL_MATH_HOST_DEVICE bool SweptBoxesSeparated(const ExactTriangle samples_a[3],
                           const ExactTriangle samples_b[3]) {
    for (unsigned component = 0; component < 3; ++component) {
      Dyadic minimum_a = Component(samples_a[0].vertex[0], component);
      Dyadic maximum_a = minimum_a;
      Dyadic minimum_b = Component(samples_b[0].vertex[0], component);
      Dyadic maximum_b = minimum_b;
      for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
        const unsigned sample = endpoint * 2;
        for (unsigned vertex = 0; vertex < 3; ++vertex) {
          const Dyadic a =
              Component(samples_a[sample].vertex[vertex], component);
          const Dyadic b =
              Component(samples_b[sample].vertex[vertex], component);
          if (Compare(a, minimum_a) < 0)
            minimum_a = a;
          if (Compare(a, maximum_a) > 0)
            maximum_a = a;
          if (Compare(b, minimum_b) < 0)
            minimum_b = b;
          if (Compare(b, maximum_b) > 0)
            maximum_b = b;
        }
      }
      if (Compare(maximum_a, minimum_b) < 0 ||
          Compare(maximum_b, minimum_a) < 0)
        return true;
    }
    return false;
  }

};

}  // namespace tlfea::contact::represented_interval_crossing::native
