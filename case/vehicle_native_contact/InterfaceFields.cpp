#include "InterfaceFields.h"
#include "case/vehicle_self_contact/native/InitializerControlsSource.h"
#include "lib_src/collision/RadiossType25Coefficients.h"
#include "lib_utils/BoundedArena.h"
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_native_contact::detail {
namespace {
using Role = vehicle_dynamics::native_contact::Role;
FieldInputs Fields(const SourceInputs& input, Role role, tl::util::ConstView<double> secondary) {
    const auto& main = input.self.main_source();
    if (role == Role::Self)
        return {input.self.snapshot(), input.wall.nodes(), main.coefficients(), main.main_gaps(),
            main.secondary_nodes(), secondary, main.secondary_gaps()};
    output::Require(role == Role::MeshWall, "Unknown case interface role");
    return {input.wall.starter(), input.wall.nodes(), input.wall.main_coefficients(), input.wall.main_gaps(),
        input.wall.secondary_nodes(), secondary, input.wall.secondary_gaps()};
}
}
InterfaceFieldForecast InterfaceFields::Preflight(const SourceInputs& input, Role role, FieldPackingLimits limits) {
    const bool self = role == Role::Self;
    output::Require(self || role == Role::MeshWall, "Unknown case interface role");
    const auto nodes = self ? input.self.main_source().secondary_nodes() : input.wall.secondary_nodes();
    const auto gap = self ? input.self.main_source().main_gaps() : input.wall.main_gaps();
    output::Require(nodes.size() <= limits.secondaries && gap.size() <= limits.mains,
                    "Interface staging counts exceed capacity");
    InterfaceFieldForecast result;
    const auto& topology = self ? input.self.snapshot() : input.wall.starter();
    result.starter_fields = FieldPacking::ForecastStorage(topology.node_count,
        topology.main_count, nodes.size(), limits).retained_bytes;
    if (!self) result.ready_fields = result.starter_fields;
    tl::util::BoundedArenaLayout budget(limits.host_bytes);
    tl::util::ArenaRegion unused;
    output::Require(budget.Append<double>(2 * gap.size(), unused) &&
                        (!self || budget.Append<double>(2 * nodes.size(), unused)),
                    "Interface scalar staging exceeds capacity");
    result.scalar_vectors = budget.bytes();
    output::Require(budget.Append<std::byte>(result.starter_fields, unused) &&
                        budget.Append<std::byte>(result.ready_fields, unused) &&
                        budget.Append<std::byte>(sizeof(InterfaceFields) + 256, unused),
                    "Complete interface field staging exceeds capacity");
    result.retained_bytes = budget.bytes();
    return result;
}
InterfaceFields InterfaceFields::Prepare(const SourceInputs& input, Role role, FieldPackingLimits limits) {
    InterfaceFields result;
    result.forecast_ = Preflight(input, role, limits);
    const bool self = role == Role::Self;
    if (self) {
        const auto roster = input.self.main_source().secondary_nodes();
        const auto global = input.wall.global_coefficients();
        const double scale = input.controls.raw_controls().stfac;
        result.secondary_coefficients_.resize(roster.size());
        output::Require(result.secondary_coefficients_.capacity() <= 2 * roster.size(),
                        "Actual secondary coefficient capacity exceeds forecast");
        for (std::size_t i = 0; i < roster.size(); ++i) {
            output::Require(roster[i] < global.size(), "Original self NSV is outside the common nodal K source");
            native::NativeScalarCoefficient value;
            const auto status = native::EvaluateNativeSecondaryCoefficient(
                {1., global[roster[i]], scale}, &value);
            output::Require(status == native::CoefficientStatus::Ok,
                            "Source-resolved original secondary coefficient rejected");
            result.secondary_coefficients_[i] = value.value;
        }
    }
    const auto secondary = self ? tl::util::ConstView<double>{result.secondary_coefficients_.data(), result.secondary_coefficients_.size()} :
        input.wall.secondary_coefficients();
    const auto fields = Fields(input, role, secondary);
    result.main_search_gaps_.resize(fields.main_gaps.size());
    output::Require(result.main_search_gaps_.capacity() <= 2 * fields.main_gaps.size(),
                    "Actual main search-gap capacity exceeds forecast");
    for (std::size_t i = 0; i < fields.main_gaps.size(); ++i)
        result.main_search_gaps_[i] = fields.main_gaps[i].maximum;
    result.starter_.emplace(FieldPacking::Starter(fields, limits));
    if (!self) result.ready_.emplace(FieldPacking::FixedReady(fields, input.wall.fixed_ready(), limits));
    return result;
}
} // namespace crash::cases::vehicle_native_contact::detail
