#include "mixed_starter/Internal.h"
namespace crash::cases::vehicle_self_contact::native::mixed_starter {
namespace d = detail;
struct MixedStarterSource::Data {
    explicit Data(const MainSource& source) : main(source) {}
    std::optional<DomainEmbedding> embedding;
    d::CombinedInput combined;
    MainSource main;
    tl::util::HostArena arena;
    s::Snapshot snapshot;
    Forecast forecast;
    Provenance provenance;
    s::NeighborWarnings warnings;
};
namespace detail {
c::Topology NormalTopology(const s::Snapshot& value) noexcept {
    c::Topology result;
    result.mains = value.mains;
    result.nodes = value.node_count;
    result.primary_count = value.primary_count;
    result.main_count = value.main_count;
    result.references = value.starter.reference_count;
    result.normal_to_main = {value.normal_offsets, value.starter.reference_count+1,
        value.normal_mains, value.normal_incidence_count};
    result.source_profile = value.profile;
    result.source_topology = value.topology;
    result.primary_roles = value.primary_roles;
    result.primary_role_count = value.primary_role_count;
    result.mixed_maps = {value.primary_to_partner, value.primary_count};
    return result;
}
}
Forecast MixedStarterSource::Preflight(const MainSource& source, Limits limits) {
    return d::Budget(source, limits);
}
Forecast MixedStarterSource::Preflight(const MainSource& source, const DomainEmbedding& embedding, Limits limits) {
    return d::Budget(source, limits, &embedding);
}
Preparation MixedStarterSource::Prepare(const MainSource& source, Limits limits) {
    return PrepareImpl(source, nullptr, limits);
}
Preparation MixedStarterSource::Prepare(const MainSource& source, const DomainEmbedding& embedding, Limits limits) {
    return PrepareImpl(source, &embedding, limits);
}
Preparation MixedStarterSource::PrepareImpl(const MainSource& source, const DomainEmbedding* embedding, Limits limits) {
    Preparation result;
    try {
        const auto forecast = d::Budget(source, limits, embedding);
        static_assert(sizeof(Data)+64 < (64u << 10));
        auto next = std::make_shared<Data>(source);
        next->forecast = forecast;
        if (embedding) {
            next->embedding.emplace(*embedding);
            next->combined = d::PackCombined(source, *embedding, forecast.combined_inputs);
        }
        const auto input = embedding ? next->combined.Input(source) : source.startup_input();
        const auto post = source.post_gapm();
        const auto prefix = d::Prefix(source);
        const auto exact = embedding ? s::PreflightMixedStarter(input, source.mixed().sides(), post, prefix, limits.numerical) :
            s::PreflightMixedStarter(input, source.mixed().sides(), post, limits.numerical);
        if (exact.status != s::Status::Ok || exact.output_bytes > forecast.starter_output ||
            exact.scratch_bytes > forecast.starter_scratch)
            d::Reject(Status::InvalidInput, "Packed combined Starter input differs from admitted storage/source prefix");
        if (!next->arena.Initialize(forecast.starter_output))
            d::Reject(Status::ResourceLimit, "Mixed Starter output allocation failed");
        {
            tl::util::HostArena scratch;
            if (!scratch.Initialize(forecast.starter_scratch))
                d::Reject(Status::ResourceLimit, "Mixed Starter scratch allocation failed");
            const auto report = embedding ? s::BuildStarter(input, source.mixed().sides(), post, prefix,
                limits.numerical, next->arena, scratch, &next->snapshot) :
                s::BuildStarter(input, source.mixed().sides(), post, limits.numerical, next->arena, scratch, &next->snapshot);
            if (report.status != s::Status::Ok) {
                result.report.status = report.status == s::Status::ResourceLimit ? Status::ResourceLimit : Status::UnsupportedSource;
                result.report.reason = "Authentic mixed post-GAPM packet failed native Starter topology";
                result.report.numerical_stage = NumericalStage::Build;
                result.report.startup_report = report;
                return result;
            }
            next->warnings = report.neighbor_warnings;
        }
        // Exact immutable origin/CSR/optional-partner admission is host-only.
        // Its bounded allocation is sequential with, not added to, TL scratch.
        const auto checked = c::ValidateMixedSource(d::NormalTopology(next->snapshot),
            next->snapshot, forecast.normal_source_validation);
        if (checked.status != c::Status::Ok) {
            result.report.status = checked.status == c::Status::ResourceLimit ? Status::ResourceLimit : Status::InvalidInput;
            result.report.reason = "Mixed Starter result failed complete source/normal-map admission";
            result.report.numerical_stage = NumericalStage::NormalSourceAdmission;
            result.report.normal_source_report = checked;
            return result;
        }
        next->provenance.source_digest = source.provenance().source_digest;
        next->provenance.post_gapm_digest = source.provenance().output_digest;
        next->provenance.output_digest = d::Digest(next->provenance, next->snapshot, input, limits.metadata_bytes);
        result.source.emplace(MixedStarterSource(std::move(next)));
        result.report = {Status::Ready, "Genuine mixed Starter topology/cache; initial history and runtime remain separate"};
    } catch (const d::Failure& failure) {
        result.report = failure.report;
        result.source.reset();
    } catch (const std::bad_alloc&) {
        result.report = {Status::ResourceLimit, "Mixed Starter source allocation failed"};
        result.source.reset();
    } catch (const std::exception& failure) {
        result.report = {Status::InvalidInput, std::string(failure.what()).substr(0, 1024)};
        result.source.reset();
    }
    return result;
}
const MainSource& MixedStarterSource::main_source() const noexcept { return data_->main; }
const s::Snapshot& MixedStarterSource::snapshot() const noexcept { return data_->snapshot; }
s::Input MixedStarterSource::startup_input() const noexcept {
    return data_->embedding ? data_->combined.Input(data_->main) : data_->main.startup_input();
}
const DomainEmbedding* MixedStarterSource::embedding() const noexcept {
    return data_->embedding ? &*data_->embedding : nullptr;
}
const tl::fea::NodalNodeDomain& MixedStarterSource::domain() const noexcept {
    return data_->embedding ? data_->embedding->domain() : d::OriginalDomain(data_->main);
}
const Forecast& MixedStarterSource::forecast() const noexcept { return data_->forecast; }
const Provenance& MixedStarterSource::provenance() const noexcept { return data_->provenance; }
const s::NeighborWarnings& MixedStarterSource::neighbor_warnings() const noexcept { return data_->warnings; }
}
