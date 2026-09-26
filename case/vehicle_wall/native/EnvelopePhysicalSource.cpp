#include "physical/Internal.h"
#include <cmath>
#include <optional>

namespace crash::cases::vehicle_wall::native {
namespace d = physical_detail;
struct EnvelopePhysicalSource::Data {
    Data(const WallSource& w, const vehicle_startup::VehicleShellReferences& r,
         const modelio::physical_scope::DomainEmbedding& e) : wall(w), references(r), embedding(e) {}
    WallSource wall;
    vehicle_startup::VehicleShellReferences references;
    modelio::physical_scope::DomainEmbedding embedding;
    tl::fea::ShellBatchBinding shells;
    EnvironmentParent environment;
    std::optional<d::model::detail::Components> components;
    EnvelopePhysicalForecast forecast;
};
EnvelopePhysicalForecast EnvelopePhysicalSource::Preflight(const WallSource& wall,
        const vehicle_startup::VehicleShellReferences& refs, EnvelopePhysicalLimits limits) {
    d::Check(wall, refs, limits);
    const auto& origin = wall.vehicle_origin();
    EnvelopePhysicalForecast f;
    f.wall_source = wall.forecast().peak_bytes;
    f.embedding = modelio::physical_scope::DomainEmbedding::Preflight(origin.source(), origin.domain(),
        wall.domain(), d::Suffix(wall), limits.embedding).peak_bytes;
    const auto shells = vehicle_startup::ForecastShellBinding(refs, limits.shells);
    // Reuse the complete existing forecast, charging one extra input and full
    // capacity slack. The native owned/scratch reservations already cover their
    // declared maxima, which Check verifies include the new QEPH and four nodes.
    tl::util::BoundedArenaLayout shell_bytes(limits.host_bytes), all(limits.host_bytes);
    tl::util::ArenaRegion region;
    d::Require(shell_bytes.Append<std::byte>(shells.total_bytes, region) &&
        shell_bytes.Append<std::byte>(shells.input_bytes, region) &&
        shell_bytes.Append<d::fe::ShellQephBindingInput>(2, region),
        "Combined shell source reservation exceeds cap");
    f.shell_binding = shell_bytes.bytes();
    // Both original contributor adapters retain the checked embedding. Charge
    // their entire current source caps before either is constructed.
    f.contributor_sources = modelio::point_mass::Limits{}.host_bytes + modelio::type25::Limits{}.host_bytes;
    const auto native = d::model::detail::NativeFootprint(origin, limits.components);
    f.native_components = native.native_reservation;
    f.component_packing = native.packing_bytes;
    f.fixed_bytes = sizeof(EnvelopePhysicalSource)+sizeof(Data)+4096;
    for (const auto bytes : {f.wall_source, f.embedding, f.shell_binding, f.contributor_sources,
            f.native_components, f.component_packing, f.fixed_bytes})
        d::Require(all.Append<std::byte>(bytes, region), "Complete envelope physical-source forecast exceeds cap");
    f.peak_bytes = all.bytes();
    return f;
}
EnvelopePhysicalSource EnvelopePhysicalSource::Prepare(const WallSource& wall,
        const vehicle_startup::VehicleShellReferences& refs, EnvelopePhysicalLimits limits) {
    const auto forecast = Preflight(wall, refs, limits);
    const auto& origin = wall.vehicle_origin();
    const auto embedding = modelio::physical_scope::DomainEmbedding::Prepare(origin.source(), origin.domain(),
        wall.domain(), d::Suffix(wall), limits.embedding);
    auto next = std::make_shared<Data>(wall, refs, embedding);
    {
        const auto inputs = d::Pack(wall, refs);
        const auto report = next->shells.InitializeFormulations(inputs.Borrow(refs.source().counts().nodes+4), limits.shells.native);
        d::Require(report.status == d::fe::ShellBindingStatus::Success, report.message);
    }
    next->components.emplace(embedding, d::model::detail::WeldDeclaration());
    d::model::detail::PrepareComponents(origin, embedding.domain(), next->shells, limits.components, *next->components);
    const auto& ledger = next->components->ledger;
    d::Require(ledger.domain() && ledger.domain()->SharesStorage(embedding.domain()) &&
        ledger.scope().uncovered_nodes == 0 && ledger.nodes().size() == wall.domain().node_count(),
        "Combined physical ledger does not cover its actual full domain");
    for (const auto& row : ledger.nodes())
        d::Require(std::isfinite(row.coefficients.mass) && row.coefficients.mass > 0,
            "Combined initial raw mass must remain finite positive on every physical node");
    auto& e = next->environment;
    e.element_id=wall.ids().shell; e.part_id=wall.ids().part;
    e.material_id=wall.ids().material; e.section_id=wall.ids().section;
    e.qeph_index=refs.counts().qeph_succeeded; e.catalog_append_ordinal=refs.rows().size();
    for (unsigned k=0; k<4; ++k) {
        e.shell_nodes[k]=next->shells.qeph_nodes(e.qeph_index)[k];
        e.domain_nodes[k]=embedding.domain().Find(wall.ids().nodes[k]);
        d::Require(e.domain_nodes[k] == wall.vehicle_prefix().nodes+k &&
            next->shells.active_nodes()[e.shell_nodes[k]].source_id == wall.ids().nodes[k],
            "Environment parent/source/domain mapping changed");
    }
    next->forecast=forecast;
    return EnvelopePhysicalSource(std::move(next));
}
const WallSource& EnvelopePhysicalSource::wall() const noexcept { return data_->wall; }
const modelio::physical_scope::DomainEmbedding& EnvelopePhysicalSource::embedding() const noexcept { return data_->embedding; }
const tl::fea::NodalNodeDomain& EnvelopePhysicalSource::domain() const noexcept { return data_->embedding.domain(); }
const vehicle_startup::VehicleShellReferences& EnvelopePhysicalSource::vehicle_references() const noexcept { return data_->references; }
const tl::fea::ShellBatchBinding& EnvelopePhysicalSource::shells() const noexcept { return data_->shells; }
const EnvironmentParent& EnvelopePhysicalSource::environment_parent() const noexcept { return data_->environment; }
const modelio::point_mass::VehiclePointMassSource& EnvelopePhysicalSource::point_masses() const noexcept { return data_->components->masses; }
const modelio::type25::VehicleType25Source& EnvelopePhysicalSource::welds() const noexcept { return data_->components->welds; }
const tl::fea::type13::Model& EnvelopePhysicalSource::beams() const noexcept { return data_->components->beams; }
const tl::fea::solids::Model& EnvelopePhysicalSource::solids() const noexcept { return data_->components->solids; }
const tl::fea::beam18::Model& EnvelopePhysicalSource::structural_beams() const noexcept { return data_->components->structural_beams; }
const tl::fea::NodalCoefficientLedger& EnvelopePhysicalSource::coefficients() const noexcept { return data_->components->ledger; }
const tl::fea::NodalRigidGroupModel& EnvelopePhysicalSource::plain_groups() const noexcept { return data_->components->plain; }
const tl::fea::NodalRigidAssemblyBinding& EnvelopePhysicalSource::rigid_assembly() const noexcept { return data_->components->rigid; }
const EnvelopePhysicalForecast& EnvelopePhysicalSource::forecast() const noexcept { return data_->forecast; }
}
