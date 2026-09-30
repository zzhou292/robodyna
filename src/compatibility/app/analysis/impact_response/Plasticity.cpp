#include "Plasticity.h"

#include "lib_utils/BoundedArena.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <set>

namespace crash::analysis::impact_response {
namespace {

using output::Require;

SampleStamp Stamp(const output::full_shell::FrameStamp& stamp) {
    return {stamp.epoch, stamp.attempt, stamp.time};
}

std::array<double, 3> Centroid(const ParentCatalogEntry& parent,
    const std::vector<double>& positions) {
    const auto count = parent.node_count();
    std::array<double, 3> result{};
    for (unsigned slot = 0; slot < count; ++slot) {
        const auto node = parent.mapped_nodes[slot];
        Require(3 * std::size_t(node) + 2 < positions.size(),
            "Impact analysis parent centroid references an absent position");
        for (unsigned axis = 0; axis < 3; ++axis)
            result[axis] += positions[3 * std::size_t(node) + axis] /
                static_cast<double>(count);
    }
    Require(std::isfinite(result[0]) && std::isfinite(result[1]) &&
            std::isfinite(result[2]),
        "Impact analysis parent centroid is unrepresentable");
    return result;
}

Occurrence At(const output::full_shell::FrameStamp& stamp, bool active,
    const std::array<double, 3>& centroid) {
    return {true, Stamp(stamp), active, centroid};
}

bool Earlier(const Witness& current, const Occurrence& candidate,
    std::uint64_t current_element, std::uint64_t candidate_element) {
    return !current.available ||
        candidate.stamp.epoch < current.occurrence.stamp.epoch ||
        (candidate.stamp.epoch == current.occurrence.stamp.epoch &&
            candidate_element < current_element);
}

void Extend(SpatialBounds& bounds, const std::array<double, 3>& value) {
    if (!bounds.available) {
        bounds.available = true;
        bounds.low_m = bounds.high_m = value;
        return;
    }
    for (unsigned axis = 0; axis < 3; ++axis) {
        bounds.low_m[axis] = std::min(bounds.low_m[axis], value[axis]);
        bounds.high_m[axis] = std::max(bounds.high_m[axis], value[axis]);
    }
}

std::size_t LayoutIndex(std::size_t points) {
    return points == 0 ? 0 : points == 1 ? 1 : points == 3 ? 2 :
        points == 4 ? 3 : 4;
}

}  // namespace

PlasticityAccumulator::PlasticityAccumulator(
    const output::full_shell::Context& context,
    std::vector<ParentCatalogEntry> catalog, AnalysisLimits limits)
    : context_(context), limits_(limits) {
    constexpr std::size_t MaximumHostBytes = 512u << 20;
    Require(limits_.samples && limits_.samples <= 10000 &&
            limits_.parts && limits_.parts <= 65536 &&
            limits_.host_bytes && limits_.host_bytes <= MaximumHostBytes &&
            limits_.report_bytes && limits_.report_bytes <= 16u << 20 &&
            catalog.size() == context_.parents().size(),
        "Impact analysis limits or parent extent are invalid");

    tl::util::BoundedArenaLayout budget(limits_.host_bytes);
    tl::util::ArenaRegion ignored;
    Require(budget.Append<ParentCatalogEntry>(catalog.size(), ignored) &&
            budget.Append<ParentPlasticity>(catalog.size(), ignored) &&
            budget.Append<std::uint8_t>(context_.points(), ignored) &&
            budget.Append<SamplePlasticity>(limits_.samples, ignored) &&
            budget.Append<PartPlasticity>(limits_.parts, ignored) &&
            budget.Append<std::optional<double>>(catalog.size(), ignored),
        "Impact analysis aggregate workspace exceeds its host cap");

    result_.catalog = std::move(catalog);
    result_.parents.resize(result_.catalog.size());
    point_ever_positive_.assign(context_.points(), 0);
    const auto& declared = context_.parents();
    const auto& offsets = context_.point_offsets();
    for (std::size_t i = 0; i < result_.catalog.size(); ++i) {
        const auto& entry = result_.catalog[i];
        const auto& parent = declared[i];
        Require(entry.source_element == parent.source_element &&
                entry.source_part == parent.source_part &&
                entry.source_elform == parent.source_elform &&
                entry.native_family == parent.native_family &&
                entry.native_points == parent.native_points &&
                entry.plastic == parent.plastic &&
                entry.node_count() >= 3 && entry.node_count() <= 4,
            "Impact analysis catalog differs from its frame declaration");
        for (unsigned slot = 0; slot < 4; ++slot)
            Require(entry.mapped_nodes[slot] < context_.nodes() &&
                    entry.source_nodes[slot],
                "Impact analysis catalog has an absent mapped/source node");

        ++result_.native_point_layouts[LayoutIndex(parent.native_points)];
        auto& state = result_.parents[i];
        state.field_available =
            parent.plastic == output::full_shell::PlasticField::NativeEquivalentPlasticStrain;
        const auto stored = offsets[i + 1] - offsets[i];
        if (state.field_available) {
            Require(stored == parent.native_points,
                "Impact analysis available parent point extent differs");
            ++result_.available_parents;
            result_.native_points += stored;
        } else {
            Require(stored == 0,
                "Impact analysis unavailable parent unexpectedly owns stored points");
            if (parent.plastic == output::full_shell::PlasticField::NotApplicable)
                ++result_.not_applicable_parents;
            else
                ++result_.unavailable_parents;
        }
    }
    Require(result_.native_points == context_.points(),
        "Impact analysis stored point total differs from Context");
}

void PlasticityAccumulator::Observe(const output::physical_run::Sample& sample) {
    namespace records = output::full_shell;
    Require(!finished_ && result_.samples.size() < limits_.samples,
        "Impact analysis sample count is exhausted or already finalized");
    const auto maxima = records::ParentPlasticMaxima(context_, sample.frame);
    const auto& activity_context = sample.activity.context();
    Require(records::SameIdentity(context_.identity(), activity_context.identity()) &&
            context_.point_layout_sha256() == activity_context.point_layout_sha256() &&
            records::SameStamp(sample.frame.stamp, sample.activity.stamp()) &&
            sample.activity.words().size() == (context_.parents().size() + 63) / 64,
        "Impact analysis activity differs from the authenticated frame");

    if (result_.samples.empty()) {
        Require(sample.frame.stamp.epoch == 0,
            "Impact analysis must start with the actual saved initial state");
    } else {
        const auto& previous = result_.samples.back().stamp;
        Require(sample.frame.stamp.epoch > previous.epoch &&
                sample.frame.stamp.attempt > previous.attempt &&
                sample.frame.stamp.time > previous.time_s,
            "Impact analysis saved samples are not strictly ordered");
    }

    SamplePlasticity current;
    current.stamp = Stamp(sample.frame.stamp);
    current.active_parents = sample.activity.active_count();
    std::set<std::uint64_t> positive_parts;
    const auto& offsets = context_.point_offsets();

    for (std::size_t i = 0; i < result_.parents.size(); ++i) {
        auto& state = result_.parents[i];
        const auto& source = result_.catalog[i];
        const bool active = sample.activity.active(i);
        state.ever_inactive |= !active;
        state.active_at_final_sample = active;

        if (result_.samples.empty())
            state.reference_centroid_m = Centroid(source, sample.frame.position_xyz);

        const auto begin = offsets[i], end = offsets[i + 1];
        std::size_t positive = 0;
        for (std::size_t point = begin; point < end; ++point) {
            if (sample.frame.plastic_points[point] > 0) {
                ++positive;
                point_ever_positive_[point] = 1;
            }
        }
        state.peak_positive_points = std::max(state.peak_positive_points, positive);
        state.final_positive_points = positive;
        state.final_max_strain = maxima[i].value_or(0);
        if (positive) {
            ++current.positive_parents;
            current.positive_points += positive;
            if (active) current.active_positive_points += positive;
            else current.inactive_positive_points += positive;
            positive_parts.insert(source.source_part);
            const auto centroid = Centroid(source, sample.frame.position_xyz);
            if (!state.ever_positive) {
                state.ever_positive = true;
                state.first_positive_strain = *maxima[i];
                state.first_positive = At(sample.frame.stamp, active, centroid);
            }
            if (*maxima[i] > state.peak_strain) {
                state.peak_strain = *maxima[i];
                state.peak = At(sample.frame.stamp, active, centroid);
            }
        }
        if (state.ever_positive)
            state.final_centroid_m = Centroid(source, sample.frame.position_xyz);

        if (positive && (!current.maximum.available ||
                *maxima[i] > current.maximum.value)) {
            current.maximum.available = true;
            current.maximum.parent_index = i;
            current.maximum.value = *maxima[i];
            current.maximum.occurrence =
                At(sample.frame.stamp, active, state.final_centroid_m);
        }
    }
    current.positive_parts = positive_parts.size();
    Require(current.active_positive_points + current.inactive_positive_points ==
            current.positive_points,
        "Impact analysis active/inactive point partition differs");
    result_.inactive_positive_history_observed |=
        current.inactive_positive_points != 0;
    result_.samples.push_back(std::move(current));
}

PlasticityResult PlasticityAccumulator::Finish() {
    Require(!finished_ && !result_.samples.empty(),
        "Impact analysis has no samples or was already finalized");
    finished_ = true;
    result_.ever_positive_points =
        std::count(point_ever_positive_.begin(), point_ever_positive_.end(), std::uint8_t{1});
    const auto& offsets = context_.point_offsets();
    for (std::size_t i = 0; i < result_.parents.size(); ++i) {
        auto& state = result_.parents[i];
        state.ever_positive_points = std::count(
            point_ever_positive_.begin() + offsets[i],
            point_ever_positive_.begin() + offsets[i + 1], std::uint8_t{1});
        if (!state.ever_positive) continue;
        ++result_.ever_positive_parents;
        const auto& source = result_.catalog[i];
        if (Earlier(result_.first_positive, state.first_positive,
                result_.first_positive.available
                    ? result_.catalog[result_.first_positive.parent_index].source_element : 0,
                source.source_element)) {
            result_.first_positive = {true, i, state.first_positive_strain,
                state.first_positive};
        }
        const auto current_peak_element = result_.peak.available
            ? result_.catalog[result_.peak.parent_index].source_element : 0;
        if (!result_.peak.available || state.peak_strain > result_.peak.value ||
            (state.peak_strain == result_.peak.value &&
                source.source_element < current_peak_element)) {
            result_.peak = {true, i, state.peak_strain, state.peak};
        }
    }

    std::map<std::uint64_t, std::size_t> indices;
    for (std::size_t i = 0; i < result_.catalog.size(); ++i) {
        const auto& source = result_.catalog[i];
        const auto& state = result_.parents[i];
        auto [where, inserted] = indices.emplace(source.source_part, result_.parts.size());
        if (inserted) {
            Require(result_.parts.size() < limits_.parts,
                "Impact analysis part count exceeds its cap");
            PartPlasticity part;
            part.source_part = source.source_part;
            part.source_material = source.source_material;
            part.source_section = source.source_section;
            part.source_elform = source.source_elform;
            result_.parts.push_back(std::move(part));
        }
        auto& part = result_.parts[where->second];
        Require(part.source_material == source.source_material &&
                part.source_section == source.source_section &&
                part.source_elform == source.source_elform,
            "Impact analysis PID has inconsistent source declarations");
        ++part.parents;
        if (std::find(part.native_families.begin(), part.native_families.end(),
                source.native_family) == part.native_families.end())
            part.native_families.push_back(source.native_family);
        if (state.field_available) {
            ++part.field_parents;
            part.native_points += source.native_points;
        }
        part.ever_inactive_parents += state.ever_inactive;
        if (!state.ever_positive) continue;
        ++part.ever_positive_parents;
        part.ever_positive_points += state.ever_positive_points;
        part.final_positive_parents += state.final_positive_points != 0;
        part.final_positive_points += state.final_positive_points;
        part.final_max_strain = std::max(part.final_max_strain, state.final_max_strain);
        Extend(part.yielded_reference_centroid_bounds, state.reference_centroid_m);
        Extend(part.yielded_final_centroid_bounds, state.final_centroid_m);

        const auto first_element = part.first_positive.available
            ? result_.catalog[part.first_positive.parent_index].source_element : 0;
        if (Earlier(part.first_positive, state.first_positive,
                first_element, source.source_element))
            part.first_positive = {true, i, state.first_positive_strain,
                state.first_positive};
        const auto peak_element = part.peak.available
            ? result_.catalog[part.peak.parent_index].source_element : 0;
        if (!part.peak.available || state.peak_strain > part.peak.value ||
            (state.peak_strain == part.peak.value &&
                source.source_element < peak_element)) {
            part.peak = {true, i, state.peak_strain, state.peak};
            part.peak_strain = state.peak_strain;
        }
    }
    for (auto& part : result_.parts)
        std::sort(part.native_families.begin(), part.native_families.end());
    std::sort(result_.parts.begin(), result_.parts.end(),
        [](const PartPlasticity& a, const PartPlasticity& b) {
            return a.source_part < b.source_part;
        });
    result_.ever_positive_parts = std::count_if(result_.parts.begin(), result_.parts.end(),
        [](const PartPlasticity& part) { return part.ever_positive_parents != 0; });
    return std::move(result_);
}

}  // namespace crash::analysis::impact_response
