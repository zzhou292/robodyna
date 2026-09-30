#include "FieldPacking.h"
#include "output/ArtifactIO.h"
#include "lib_utils/BoundedArena.h"
#include "lib_src/collision/radioss_type25/search/Ranges.h"
namespace crash::cases::vehicle_native_contact::detail {
namespace {
template<class T> bool Span(tl::util::ConstView<T> values) {
    return native::search::detail::Span(values.data(), values.size());
}
}
FieldPackingForecast FieldPacking::ForecastStorage(std::size_t nodes, std::size_t mains,
                                                  std::size_t secondaries, FieldPackingLimits limits) {
    using output::Require;
    const FieldPackingLimits hard;
    Require(limits.nodes && limits.nodes <= hard.nodes && limits.mains && limits.mains <= hard.mains &&
                limits.secondaries && limits.secondaries <= hard.secondaries &&
                limits.host_bytes && limits.host_bytes <= hard.host_bytes && nodes && nodes <= limits.nodes &&
                mains && mains <= limits.mains && secondaries && secondaries <= limits.secondaries,
            "Contact field packing count/cap is outside its bounded profile");
    tl::util::BoundedArenaLayout arena(limits.host_bytes);
    tl::util::ArenaRegion unused;
    Require(arena.Append<std::byte>(sizeof(FieldPacking) + 256, unused) &&
                arena.Append<lifecycle::Main>(2 * mains, unused) &&
                arena.Append<lifecycle::Secondary>(2 * secondaries, unused),
            "Contact field packing exceeds its host cap");
    return {arena.bytes()};
}
FieldPackingForecast FieldPacking::Preflight(const FieldInputs& in, FieldPackingLimits limits) {
    const auto& top = in.topology;
    const auto secondaries = in.secondary_nodes.size();
    const auto result = ForecastStorage(top.node_count, top.main_count, secondaries, limits);
    output::Require(top.source_generation && top.primary_count && top.primary_count <= top.main_count &&
                        in.nodes.size() == top.node_count && in.main_coefficients.size() == top.main_count &&
                        in.main_gaps.size() == top.main_count && in.secondary_coefficients.size() == secondaries &&
                        in.secondary_gaps.size() == secondaries && Span(in.nodes) && Span(in.main_coefficients) &&
                        Span(in.main_gaps) && Span(in.secondary_nodes) && Span(in.secondary_coefficients) &&
                        Span(in.secondary_gaps) && native::search::detail::Span(top.mains, top.main_count),
                    "Contact field packing source extents differ");
    return result;
}
FieldPacking FieldPacking::Starter(const FieldInputs& in, FieldPackingLimits limits) {
    return Pack(in, in.topology.starter, NormalPhase::StarterBeforeInitialContact, limits);
}
FieldPacking FieldPacking::FixedReady(const FieldInputs& in, const native::startup::FixedMainView& ready,
                                     FieldPackingLimits limits) {
    output::Require(ready.source_generation == in.topology.source_generation &&
                        ready.profile == in.topology.profile && ready.topology == in.topology.topology,
                    "Fixed-ready cache belongs to a different source generation/profile");
    return Pack(in, ready.normals, NormalPhase::FixedReady, limits);
}
FieldPacking FieldPacking::Pack(const FieldInputs& in, const native::startup::NormalView& normals,
                               NormalPhase phase, FieldPackingLimits limits) {
    const auto forecast = Preflight(in, limits);
    const auto& top = in.topology;
    output::Require(normals.reference_count == top.starter.reference_count &&
                        normals.reference_count <= 4 * top.main_count &&
                        top.normal_incidence_count <= 4 * top.main_count &&
                        native::search::detail::Span(normals.face_normals, 4 * top.main_count) &&
                        native::search::detail::Span(normals.references, normals.reference_count) &&
                        native::search::detail::Span(top.normal_offsets, normals.reference_count + 1) &&
                        native::search::detail::Span(top.normal_mains, top.normal_incidence_count),
                    "Contact field packing normal/CSR descriptors differ");
    FieldPacking result;
    result.forecast_ = forecast;
    result.phase_ = phase;
    result.mains_.resize(top.main_count);
    result.secondary_.resize(in.secondary_nodes.size());
    output::Require(result.mains_.capacity() <= 2 * top.main_count &&
                        result.secondary_.capacity() <= 2 * in.secondary_nodes.size(),
                    "Actual contact field capacity exceeds forecast");
    for (std::size_t i = 0; i < top.main_count; ++i) {
        const auto& source = top.mains[i];
        const auto& gap = in.main_gaps[i];
        auto& row = result.mains_[i];
        row.global_id = source.global_id;
        row.segment_type = source.segment_type;
        row.coefficient = in.main_coefficients[i];
        row.maximum_gap = gap.maximum;
        for (unsigned corner = 0; corner < 4; ++corner) {
            row.nodes[corner] = source.nodes[corner];
            row.normal_reference[corner] = source.normal_reference[corner];
            row.neighbors[corner] = source.neighbors[corner];
            row.normal_slot[corner] = normals.face_normals[4 * i + corner];
            row.gap[corner] = gap.corner[corner];
        }
    }
    for (std::size_t i = 0; i < in.secondary_nodes.size(); ++i) {
        output::Require(in.secondary_nodes[i] < top.node_count, "Contact secondary node is outside the actual domain");
        // Fresh input to the selected INACTI5 initializer. This is never a
        // runtime history initialization or a replacement for its DeviceSeed.
        result.secondary_[i] = {in.secondary_nodes[i], in.secondary_coefficients[i], in.secondary_gaps[i], 0};
    }
    result.borrowed_.nodes = in.nodes.data();
    result.borrowed_.node_count = in.nodes.size();
    result.borrowed_.normal_count = normals.reference_count;
    result.borrowed_.normals = normals.references;
    result.borrowed_.normal_to_main = {top.normal_offsets, normals.reference_count + 1,
        top.normal_mains, top.normal_incidence_count};
    result.borrowed_.generation = top.source_generation;
    return result;
}
lifecycle::SourceView FieldPacking::view() const noexcept {
    auto result = borrowed_;
    result.mains = mains_.data();
    result.main_count = mains_.size();
    result.secondary = secondary_.data();
    result.secondary_count = secondary_.size();
    return result;
}
} // namespace crash::cases::vehicle_native_contact::detail
