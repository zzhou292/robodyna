#pragma once
#include "PostGapmMainSource.h"
#include "modelio/physical_scope/DomainEmbedding.h"
#include "lib_src/collision/RadiossType25CurrentNormals.h"

namespace crash::cases::vehicle_self_contact::native::mixed_starter {
namespace s = tlfea::contact::radioss_type25::startup;
namespace c = tlfea::contact::radioss_type25::current_normals;
using MainSource = post_gapm::PostGapmMainSource;
using DomainEmbedding = modelio::physical_scope::DomainEmbedding;
enum class Status { Ready, InvalidInput, ResourceLimit, UnsupportedSource };
enum class Stage { StarterTopologyBeforeInitialContactHistory };
enum class NumericalStage { None, Forecast, Build, NormalSourceAdmission };
struct Report {
    Status status = Status::InvalidInput;
    std::string reason;
    NumericalStage numerical_stage = NumericalStage::None;
    s::Report startup_report;
    c::Report normal_source_report;
};
struct Limits {
    std::size_t host_bytes = std::size_t{19} << 29;
    std::size_t metadata_bytes = 1u << 20;
    s::Limits numerical;
};
struct Forecast {
    std::size_t prior_construction_peak = 0, upstream_retained = 0;
    std::size_t embedding_additional = 0, embedding_prior_peak = 0, combined_inputs = 0;
    std::size_t starter_output = 0, starter_scratch = 0, normal_source_validation = 0;
    std::size_t retained_metadata = 0, digest_and_report = 0;
    std::size_t current_phase = 0, peak_bytes = 0, retained_bytes = 0;
};
struct Provenance {
    Stage stage = Stage::StarterTopologyBeforeInitialContactHistory;
    std::string source_digest, post_gapm_digest, output_digest;
};
struct Preparation;
// Immutable source topology and genuine Starter I25NORM cache. Engine activity,
// first contact history, tied removal, cache ownership and physical publication
// remain separate stages. No all-active Engine cache is fabricated here.
class MixedStarterSource {
  public:
    MixedStarterSource(const MixedStarterSource&) noexcept = default;
    MixedStarterSource(MixedStarterSource&& other) noexcept : data_(other.data_) {}
    MixedStarterSource& operator=(const MixedStarterSource&) = delete;
    static Forecast Preflight(const MainSource&, Limits = {});
    static Preparation Prepare(const MainSource&, Limits = {});
    static Forecast Preflight(const MainSource&, const DomainEmbedding&, Limits = {});
    static Preparation Prepare(const MainSource&, const DomainEmbedding&, Limits = {});
    const MainSource& main_source() const noexcept;
    const s::Snapshot& snapshot() const noexcept;
    s::Input startup_input() const noexcept;
    const DomainEmbedding* embedding() const noexcept;
    const tl::fea::NodalNodeDomain& domain() const noexcept;
    const Forecast& forecast() const noexcept;
    const Provenance& provenance() const noexcept;
    const s::NeighborWarnings& neighbor_warnings() const noexcept;
  private:
    struct Data;
    static Preparation PrepareImpl(const MainSource&, const DomainEmbedding*, Limits);
    explicit MixedStarterSource(std::shared_ptr<const Data> value) : data_(std::move(value)) {}
    std::shared_ptr<const Data> data_;
};
struct Preparation {
    Report report;
    std::optional<MixedStarterSource> source;
};
output::Document ForecastDocument(const Forecast&);
output::Document ResultDocument(const Preparation&, std::size_t cap = 1u << 20);
}
