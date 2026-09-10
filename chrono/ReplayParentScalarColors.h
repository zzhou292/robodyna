#pragma once

#include "output/AcceptedReplay.h"
#include "ReplayGeometryLimits.h"
#include "chrono/assets/ChColor.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <utility>
#include <vector>

namespace crash::visual {

// A fixed blue/yellow/red ramp shared by mesh colors and the visible legend.
// Normalized values outside the declared range saturate at its endpoints.
inline chrono::ChColor ReplayScalarColor(double normalized) {
    const chrono::ChColor blue(.12f, .64f, .94f), yellow(.98f, .84f, .16f), red(.90f, .12f, .10f);
    const double t = std::clamp(normalized, 0., 1.);
    const auto& a = t <= .5 ? blue : yellow;
    const auto& b = t <= .5 ? yellow : red;
    return chrono::ChColor::Interp(a, b, t <= .5 ? 2*t : 2*t-1);
}

// Immutable source-parent association with a bounded, allocation-free update.
// Both display triangles of a Q4 parent necessarily index the same scalar.
// The caller stages these colors beside its geometry before publishing either.
class ReplayParentScalarColors {
  public:
    bool Initialize(const std::vector<std::uint64_t>& triangle_parents,
                    const std::vector<output::ReplayParentScalar>& field, double maximum,
                    std::vector<chrono::ChColor>& colors, ReplayGeometryLimits limits = {}) {
        if (!limits.valid() || !std::isfinite(maximum) || maximum <= 0 || triangle_parents.empty() ||
            triangle_parents.size() > limits.triangles || field.empty() || field.size() > limits.parents) return false;
        ReplayParentScalarColors next;
        next.maximum_ = maximum;
        std::map<std::uint64_t, std::size_t> parent_index;
        for (const auto& parent : field) {
            if (!parent.source_parent || !parent_index.emplace(parent.source_parent, next.parents_.size()).second)
                return false;
            next.parents_.push_back(parent.source_parent);
        }
        std::vector<bool> used(field.size(), false);
        for (auto parent : triangle_parents) {
            const auto found = parent_index.find(parent);
            if (found == parent_index.end()) return false;
            next.triangle_parent_.push_back(found->second);
            used[found->second] = true;
        }
        if (std::find(used.begin(), used.end(), false) != used.end()) return false;
        std::vector<chrono::ChColor> staged(triangle_parents.size());
        if (!next.Stage(field, staged)) return false;
        *this = std::move(next);
        colors.swap(staged);
        return true;
    }
    bool Stage(const std::vector<output::ReplayParentScalar>& field,
               std::vector<chrono::ChColor>& colors) const noexcept {
        if (maximum_ <= 0 || field.size() != parents_.size() || colors.size() != triangle_parent_.size()) return false;
        for (std::size_t i = 0; i < field.size(); ++i)
            if (field[i].source_parent != parents_[i] || !std::isfinite(field[i].value) || field[i].value < 0) return false;
        for (std::size_t t = 0; t < triangle_parent_.size(); ++t) {
            const double value = field[triangle_parent_[t]].value;
            colors[t] = ReplayScalarColor(value >= maximum_ ? 1. : value/maximum_);
        }
        return true;
    }
  private:
    double maximum_ = 0;
    std::vector<std::uint64_t> parents_;
    std::vector<std::size_t> triangle_parent_;
};
} // namespace crash::visual
