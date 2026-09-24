// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Geometry.h"
namespace tlfea::contact::represented_interval_crossing::native {
template <unsigned Bits, class IntegerPolicy = BoostIntegerPolicy<Bits>>
struct CellKernel : Geometry<Bits, IntegerPolicy> {
  using Base = Geometry<Bits, IntegerPolicy>;
  using Base::Base;
  using Base::Healthy;
  using Base::context_;
  using Base::integers_;
  using typename Base::Dyadic;
  using typename Base::ExactVec3;
  using typename Base::ExactTriangle;
  using typename Base::StaticIntersection;
  using Base::Normal;
  using Base::Degenerate;
  using Base::RegularCell;
  using Base::Component;
  using Base::Subtract;
  using Base::Dot;
  using Base::Compare;
  using Base::Zero;
  using Base::At;
  using Base::Intersects;
  using Base::IntersectionFeature;
  using Base::SweptBoxesSeparated;
  using Base::CommonTranslation;
  struct ExactScratch {
    ExactTriangle a[3];
    ExactTriangle b[3];
    ExactVec3 normal_a[3];
    ExactVec3 normal_b[3];
    bool ready_a[3]{};
    bool ready_b[3]{};

    void BeginCell() noexcept {
      std::fill_n(ready_a, 3, false);
      std::fill_n(ready_b, 3, false);
    }

    const ExactVec3& NormalAt(CellKernel& kernel, bool second, unsigned sample, NormalCounters* counters) {
      auto& ready = second ? ready_b[sample] : ready_a[sample];
      auto& normal = second ? normal_b[sample] : normal_a[sample];
      if (!ready) {
        // Publish readiness only after the original checked arithmetic succeeds.
        normal = kernel.Normal(second ? b[sample] : a[sample], counters);
        if (!kernel.Healthy()) return normal;
        ready = true;
      } else {
        CountNormalOperation(counters, &NormalCounters::cache_hits);
      }
      return normal;
    }

    bool Regular(CellKernel& kernel, bool second, NormalCounters* counters) {
      // Preserve original first-use order, including its exception boundary.
      NormalAt(kernel, second, 0, counters);
      NormalAt(kernel, second, 1, counters);
      NormalAt(kernel, second, 2, counters);
      return kernel.RegularCell(second ? normal_b : normal_a);
    }
  };

  struct ProjectionHull { Dyadic minimum, maximum; };

  template <class Project>
  ProjectionHull RelativeEndpointHull(
      const ExactTriangle samples[3], const ExactTriangle reference[3],
      unsigned anchor, const Project& project) {
    ProjectionHull result;
    bool first = true;
    for (unsigned endpoint : {0u, 2u})
      for (const auto& vertex : samples[endpoint].vertex) {
        const auto value = project(vertex, reference[endpoint].vertex[anchor]);
        if (first) { result.minimum = result.maximum = value; first = false; }
        else {
          if (Compare(value, result.minimum) < 0) result.minimum = value;
          if (Compare(value, result.maximum) > 0) result.maximum = value;
        }
      }
    return result;
  }

  bool StrictHullGap(const ProjectionHull& a, const ProjectionHull& b) {
    return Compare(a.maximum, b.minimum) < 0 || Compare(b.maximum, a.minimum) < 0;
  }

  bool RelativeCoordinatesSeparated(const ExactScratch& scratch, unsigned anchor) {
    for (unsigned coordinate = 0; coordinate < 3; ++coordinate) {
      const auto project = [this, coordinate](const ExactVec3& point, const ExactVec3& reference) {
        return Subtract(Component(point, coordinate), Component(reference, coordinate));
      };
      const auto first = RelativeEndpointHull(scratch.a, scratch.a, anchor, project);
      const auto second = RelativeEndpointHull(scratch.b, scratch.a, anchor, project);
      if (StrictHullGap(first, second)) return true;
    }
    return false;
  }

  bool RelativeAxisSeparated(const ExactScratch& scratch, unsigned anchor, const ExactVec3& axis) {
    if (Zero(axis)) return false;
    const auto project = [this, &axis](const ExactVec3& point, const ExactVec3& reference) {
      return Dot(Subtract(point, reference), axis);
    };
    const auto first = RelativeEndpointHull(scratch.a, scratch.a, anchor, project);
    const auto second = RelativeEndpointHull(scratch.b, scratch.a, anchor, project);
    return StrictHullGap(first, second);
  }

  enum class CellDisposition : std::uint8_t {
    Separated,
    Crossing,
    Split,
    Unresolved,
  };

  struct CellEvaluation {
    CellDisposition disposition = CellDisposition::Split;
    RepresentedIntervalReason reason = RepresentedIntervalReason::None;
    RepresentedIntervalResult crossing;
  };

  template <NormalReuse reuse = NormalReuse::Memoize,
            SeparationProof separation = SeparationProof::RelativeFaces,
            ExactPathReuse path_reuse = ExactPathReuse::Optimized,
            bool single_sample = false,
            CommonPointReuse point_reuse = CommonPointReuse::Original>
  CellEvaluation EvaluateCell(const RepresentedTrianglePath& path_a,
                              const RepresentedTrianglePath& path_b,
                              const RepresentedIntervalPairKey& key, Cell cell,
                              ExactScratch* scratch,
                              NormalCounters* counters = nullptr,
                              const ProjectionDomain* domain = nullptr,
                              unsigned anchor = 0,
                              SeparationCounters* separation_counters = nullptr,
                              ExactPathCounters* path_counters = nullptr,
                              CommonPointCounters* common_point_counters = nullptr) {
    static_assert(point_reuse == CommonPointReuse::Original || reuse == NormalReuse::Memoize);
    static_assert(!single_sample ||
        (separation == SeparationProof::LegacyAabb && path_reuse == ExactPathReuse::Optimized));
    scratch->BeginCell();
    CountNormalOperation(counters, &NormalCounters::evaluated_cells);
    const auto degenerate_at = [&](bool second, unsigned sample) {
      if constexpr (reuse == NormalReuse::Memoize)
        return Zero(scratch->NormalAt(*this, second, sample, counters));
      else
        return Degenerate(second ? scratch->b[sample] : scratch->a[sample], counters);
    };
    const DyadicTime times[3] = {Lower(cell), Middle(cell), Upper(cell)};
    constexpr unsigned sample_count = single_sample ? 1 : 3;
    if constexpr (single_sample)
      CountExactPathOperation(path_counters, &ExactPathCounters::single_sample_intervals);
    bool degenerate = false;
    for (unsigned sample = 0; sample < sample_count; ++sample) {
      scratch->a[sample] = At(path_a, times[sample]);
      scratch->b[sample] = At(path_b, times[sample]);
      degenerate = degenerate || degenerate_at(false, sample) ||
                   degenerate_at(true, sample);
    }
    for (unsigned sample = 0; sample < sample_count; ++sample) {
      if (degenerate_at(false, sample) ||
          degenerate_at(true, sample))
        continue;
      bool common_endpoint = false;
      if constexpr (point_reuse == CommonPointReuse::Optimized)
        if (domain && domain->eligible())
          common_endpoint = CommonEndpointPoint(path_a, path_b, times[sample], common_point_counters);
      StaticIntersection intersection;
      if constexpr (reuse == NormalReuse::Memoize) {
        const auto& normal_a = scratch->NormalAt(*this, false, sample, counters);
        const auto& normal_b = scratch->NormalAt(*this, true, sample, counters);
        intersection = Intersects(scratch->a[sample], scratch->b[sample], normal_a, normal_b,
                                 common_endpoint, common_point_counters);
      } else {
        intersection = Intersects(scratch->a[sample], scratch->b[sample], counters);
      }
      if (!intersection.intersects)
        continue;
      CellEvaluation evaluation;
      evaluation.disposition = CellDisposition::Crossing;
      auto& result = evaluation.crossing;
      result.key = key;
      const bool skip_dominated_edges = path_reuse == ExactPathReuse::Optimized &&
          domain && domain->eligible();
      if constexpr (reuse == NormalReuse::Memoize) {
        const auto& normal_a = scratch->NormalAt(*this, false, sample, counters);
        const auto& normal_b = scratch->NormalAt(*this, true, sample, counters);
        result.feature = IntersectionFeature(path_a, path_b,
            scratch->a[sample], scratch->b[sample], &normal_a, &normal_b, counters,
            skip_dominated_edges, path_counters);
      } else {
        result.feature = IntersectionFeature(path_a, path_b,
            scratch->a[sample], scratch->b[sample], nullptr, nullptr, counters, skip_dominated_edges, path_counters);
      }
      result.classification =
          RepresentedIntervalClassification::CertifiedCrossingContact;
      result.reason = RepresentedIntervalReason::None;
      result.geometry =
          intersection.coplanar ? RepresentedIntersectionGeometry::Coplanar
                                : RepresentedIntersectionGeometry::Transverse;
      result.witness_time_numerator = times[sample].numerator;
      result.witness_time_depth = times[sample].depth;
      return evaluation;
    }
    if (degenerate)
      return {CellDisposition::Unresolved,
              RepresentedIntervalReason::DegenerateGeometry, {}};
    // This specialization is reachable only after exact common-translation
    // authentication and arithmetic-domain admission. Static nonintersection and
    // nondegeneracy are then invariant over the entire represented interval.
    if constexpr (single_sample)
      return {CellDisposition::Separated, RepresentedIntervalReason::None, {}};
    const auto regular = [&](bool second) {
      if constexpr (reuse == NormalReuse::Memoize)
        return scratch->Regular(*this, second, counters);
      else
        return RegularCell(second ? scratch->b : scratch->a, counters);
    };
    if (regular(false) && regular(true)) {
      if (SweptBoxesSeparated(scratch->a, scratch->b))
        return {CellDisposition::Separated, RepresentedIntervalReason::None, {}};
      if constexpr (separation == SeparationProof::RelativeFaces) {
        if (!domain || !domain->eligible()) {
          CountSeparationOperation(separation_counters, &SeparationCounters::domain_fallback_cells);
          return {};
        }
        CountSeparationOperation(separation_counters, &SeparationCounters::eligible_cells);
        // All relative vertex projections are affine over this cell. Subtracting
        // the same canonical anchor path preserves simultaneous intersection.
        // Strictly disjoint endpoint hulls therefore certify the entire cell;
        // equality/touching never succeeds. Original sampled checks and both
        // whole-cell nondegeneracy proofs above remain mandatory.
        if (RelativeCoordinatesSeparated(*scratch, anchor)) {
          CountSeparationOperation(separation_counters, &SeparationCounters::relative_aabb_separated);
          return {CellDisposition::Separated, RepresentedIntervalReason::None, {}};
        }
        // These two axes are fixed at the lower sample. Facet order is canonical;
        // winding only reverses an axis and cannot change a symmetric strict gap.
        for (unsigned side = 0; side < 2; ++side) {
          bool separated = false;
          if constexpr (reuse == NormalReuse::Memoize) {
            const auto& axis = scratch->NormalAt(*this, side != 0, 0, counters);
            separated = RelativeAxisSeparated(*scratch, anchor, axis);
          } else {
            const auto axis = Normal(side ? scratch->b[0] : scratch->a[0], counters);
            separated = RelativeAxisSeparated(*scratch, anchor, axis);
          }
          if (separated) {
            CountSeparationOperation(separation_counters, side
                ? &SeparationCounters::second_face_separated : &SeparationCounters::first_face_separated);
            return {CellDisposition::Separated, RepresentedIntervalReason::None, {}};
          }
        }
      }
    }
    return {};
  }

  void RaiseReason(RepresentedIntervalReason candidate,
                   RepresentedIntervalReason* current) noexcept {
    if (ReasonPriority(candidate) > ReasonPriority(*current))
      *current = candidate;
  }

  template <NormalReuse reuse = NormalReuse::Memoize,
            SeparationProof separation = SeparationProof::RelativeFaces,
            ExactPathReuse path_reuse = ExactPathReuse::Optimized,
            CommonPointReuse point_reuse = CommonPointReuse::Optimized>
  RepresentedIntervalResult CertifyPair(
      const RepresentedTrianglePath& a, const RepresentedTrianglePath& b,
      RepresentedIntervalLimits limits, RepresentedIntervalPairKey key,
      Cell* dfs, std::size_t dfs_capacity, ExactScratch* scratch,
      NormalCounters* counters = nullptr,
      SeparationCounters* separation_counters = nullptr,
      ExactPathCounters* path_counters = nullptr,
      CommonPointCounters* common_point_counters = nullptr) noexcept {
    context_.BeginPair();
    if (a.motion != RepresentedMotion::LinearNodalV1 ||
        b.motion != RepresentedMotion::LinearNodalV1)
      return Unresolved(key, RepresentedIntervalReason::UnsupportedMotion, 0);
    std::size_t work = 0;
    return integers_.Protect([&]() -> RepresentedIntervalResult {
      std::size_t dfs_size = 0;
      bool all_leaves_separated = true;
      RepresentedIntervalReason unresolved = RepresentedIntervalReason::None;
      dfs[dfs_size++] = {};
      // A bit-exact common translation preserves every relative point,
      // segment and triangle predicate over the complete represented interval.
      // Test the exact binary64-real displacements rather than rounded double
      // differences: a single static exact evaluation is then a whole-interval
      // certificate, even when the absolute swept AABBs overlap.
      const bool common_translation = CommonTranslation(a, b);
      if (!Healthy()) return Unresolved(key, RepresentedIntervalReason::ExactArithmeticRange, work);
      if (common_translation) {
        CellEvaluation evaluation;
        if constexpr (path_reuse == ExactPathReuse::Optimized) {
          const auto domain = ProjectionDomain::FromPaths(a, b, limits.max_depth);
          if (domain.eligible()) {
            evaluation = EvaluateCell<reuse, SeparationProof::LegacyAabb, path_reuse, true, point_reuse>(
                a, b, key, {}, scratch, counters, &domain, 0, nullptr, path_counters, common_point_counters);
          } else {
            evaluation = EvaluateCell<reuse, SeparationProof::LegacyAabb, ExactPathReuse::Original, false, point_reuse>(
                a, b, key, {}, scratch, counters, nullptr, 0, nullptr, path_counters, common_point_counters);
          }
        } else {
          evaluation = EvaluateCell<reuse, SeparationProof::LegacyAabb, ExactPathReuse::Original, false, point_reuse>(
              a, b, key, {}, scratch, counters, nullptr, 0, nullptr, path_counters, common_point_counters);
        }
        if (!Healthy()) return Unresolved(key, RepresentedIntervalReason::ExactArithmeticRange, work);
        if (evaluation.disposition == CellDisposition::Crossing) {
          evaluation.crossing.geometry =
              evaluation.crossing.geometry == RepresentedIntersectionGeometry::Coplanar
                  ? RepresentedIntersectionGeometry::ExactCommonTranslationCoplanar
                  : RepresentedIntersectionGeometry::ExactCommonTranslationTransverse;
          evaluation.crossing.work = 1;
          return evaluation.crossing;
        }
        if (evaluation.disposition == CellDisposition::Unresolved)
          return Unresolved(key, evaluation.reason, 1);
        RepresentedIntervalResult result;
        result.key = key;
        result.classification =
            RepresentedIntervalClassification::CertifiedSeparated;
        result.reason = RepresentedIntervalReason::None;
        result.work = 1;
        return result;
      }
      const auto domain = ProjectionDomain::FromPaths(a, b, limits.max_depth);
      const auto anchor = CanonicalAnchor(a);
      while (dfs_size) {
        if (work >= limits.max_work_per_pair) {
          all_leaves_separated = false;
          RaiseReason(RepresentedIntervalReason::WorkExhausted, &unresolved);
          break;
        }
        const Cell cell = dfs[--dfs_size];
        ++work;
        auto evaluation = EvaluateCell<reuse, separation, path_reuse, false, point_reuse>(a, b, key, cell, scratch, counters,
                                                         &domain, anchor, separation_counters, path_counters, common_point_counters);
        if (!Healthy()) return Unresolved(key, RepresentedIntervalReason::ExactArithmeticRange, work);
        if (evaluation.disposition == CellDisposition::Crossing) {
          evaluation.crossing.work = work;
          return evaluation.crossing;
        }
        if (evaluation.disposition == CellDisposition::Separated)
          continue;
        if (evaluation.disposition == CellDisposition::Unresolved) {
          all_leaves_separated = false;
          RaiseReason(evaluation.reason, &unresolved);
          continue;
        }
        if (cell.depth >= limits.max_depth) {
          all_leaves_separated = false;
          RaiseReason(RepresentedIntervalReason::WorkExhausted, &unresolved);
          continue;
        }
        const Cell right{cell.lower + cell.upper, cell.upper * 2,
                         cell.depth + 1};
        const Cell left{cell.lower * 2, cell.lower + cell.upper,
                        cell.depth + 1};
        if (dfs_size + 2 > dfs_capacity) {
          all_leaves_separated = false;
          RaiseReason(RepresentedIntervalReason::ExactArithmeticRange,
                      &unresolved);
          break;
        }
        dfs[dfs_size++] = right;
        dfs[dfs_size++] = left;
      }

      if (all_leaves_separated) {
        RepresentedIntervalResult result;
        result.key = key;
        result.classification =
            RepresentedIntervalClassification::CertifiedSeparated;
        result.reason = RepresentedIntervalReason::None;
        result.work = work;
        return result;
      }
      if (unresolved == RepresentedIntervalReason::None)
        unresolved = RepresentedIntervalReason::WorkExhausted;
      return Unresolved(key, unresolved, work);
    }, [&]() {
      return Unresolved(key, RepresentedIntervalReason::ExactArithmeticRange, work);
    });
  }

};

}  // namespace tlfea::contact::represented_interval_crossing::native
