#include "TiedRemovalSource.h"
#include "case/vehicle_self_contact/native/InitializerControlsSource.h"
#include "output/ArtifactIO.h"
#include "lib_utils/BoundedArena.h"
#include <limits>

namespace crash::cases::vehicle_native_contact {
namespace controls = vehicle_self_contact::native::initial_controls;
namespace {
using output::Require;
const controls::Interface& Interface(const TiedRemovalSource::Controls& source,
                                    const vehicle_startup::TiedSearchPostKinChk& post) {
    const controls::Interface* match = nullptr;
    const auto& original = post.receipt().original_contact;
    for (const auto& row : source.interfaces()) {
        if (row.kind != controls::InterfaceKind::Type2) continue;
        Require(!match, "Prepared owner contains one TYPE2 source; complete table declares more");
        Require(row.origin == controls::InterfaceOrigin::OriginalDefinition && row.native_id &&
                    row.native_storage_ordinal && row.source.filename == original.filename &&
                    row.source.keyword == original.keyword && row.source.sha256 == original.sha256 &&
                    row.source.first_line == original.first_line && row.source.last_line == original.last_line,
                "Finalized TYPE2 source block differs from the authenticated interface table");
        match = &row;
    }
    Require(match != nullptr, "Finalized owner TYPE2 source is absent from the complete table");
    return *match;
}
std::uint32_t DomainNode(const tl::fea::NodalNodeDomain& domain, std::uint64_t id) {
    const auto index = domain.Find(id);
    Require(index != SIZE_MAX && index <= UINT32_MAX,
            "Finalized TYPE2 node is absent from the actual complete owner domain");
    return static_cast<std::uint32_t>(index);
}
}
struct TiedRemovalSource::Data {
    explicit Data(const vehicle_startup::TiedCinAttachments& value) : attachments(value) {}
    vehicle_startup::TiedCinAttachments attachments;
    TiedRemovalForecast forecast;
    std::vector<tied_removal::Main> mains;
    std::vector<tied_removal::Row> rows;
    tied_removal::Interface interface;
};
TiedRemovalForecast TiedRemovalSource::Preflight(const vehicle_wall::native::EnvelopeOwnerSource& owner,
                                               const Controls& controls, TiedRemovalLimits limits) {
    const TiedRemovalLimits hard;
    Require(limits.host_bytes && limits.host_bytes <= hard.host_bytes &&
                limits.mains && limits.mains <= hard.mains && limits.rows && limits.rows <= hard.rows &&
                limits.canonical_nodes && limits.canonical_nodes <= hard.canonical_nodes,
            "Invalid TYPE2 initializer source capacity");
    const auto& attachments = owner.attachments();
    const auto& model = attachments.model();
    const auto& post = attachments.post_kinchk();
    const auto& finalized = post.classification().finalized();
    const auto& geometry = finalized.assessment().geometry();
    const auto& declaration = geometry.packing().declaration();
    const auto& canonical = declaration.canonical().data();
    const auto& actual = owner.execution_source().mechanical();
    Require(owner.physical().prepared() && model.prepared() && model.domain() &&
                model.domain()->SharesStorage(*owner.physical().domain()) &&
                model.classification() && model.classification()->SharesStorage(post.result()) &&
                &canonical == &actual.vehicle_references().source().canonical().data() &&
                &canonical == &controls.main().mixed().initial().context().pre_correction().physical()
                    .shell_source().references().source().canonical().data(),
            "TYPE2 initializer source does not share the genuine owner and canonical backing");
    Require(post.phase() == vehicle_startup::TiedPostKinChkPhase::ObservedKinetAfterKinChk &&
                finalized.receipt().level == 28 && finalized.receipt().type2_count == 1,
            "Finalized TYPE2 phase/profile is unavailable");
    (void)Interface(controls, post);
    TiedRemovalForecast result;
    result.mains = geometry.data().masters.size();
    result.rows = finalized.data().slaves.size();
    result.canonical_nodes = canonical.canonical_nodes;
    Require(result.mains && result.mains <= limits.mains && result.mains <= INT32_MAX &&
                result.rows && result.rows <= limits.rows && result.canonical_nodes <= limits.canonical_nodes &&
                result.rows == finalized.data().selected_masters.size() &&
                result.rows == post.classification().data().slaves.size() &&
                result.rows == post.result().slaves().count && result.rows == model.rows().count,
            "Complete compact TYPE2 row/IRECT extents differ");
    tl::util::BoundedArenaLayout arena(limits.host_bytes);
    tl::util::ArenaRegion unused;
    Require(arena.Append<std::byte>(sizeof(Data) + sizeof(TiedRemovalSource) + 256, unused) &&
                arena.Append<tied_removal::Main>(2 * result.mains, unused) &&
                arena.Append<tied_removal::Row>(2 * result.rows, unused),
            "Retained TYPE2 initializer source exceeds its cap");
    result.retained_bytes = arena.bytes();
    Require(arena.Append<std::uint64_t>(2 * result.canonical_nodes, unused),
            "TYPE2 source NID decode exceeds its cap");
    result.peak_bytes = arena.bytes();
    result.temporary_bytes = result.peak_bytes - result.retained_bytes;
    return result;
}
TiedRemovalSource TiedRemovalSource::Prepare(const vehicle_wall::native::EnvelopeOwnerSource& owner,
                                           const Controls& controls, TiedRemovalLimits limits) {
    const auto forecast = Preflight(owner, controls, limits);
    const auto& attachments = owner.attachments();
    const auto& post = attachments.post_kinchk();
    const auto& classified = post.classification();
    const auto& finalized = classified.finalized();
    const auto& geometry = finalized.assessment().geometry();
    const auto& packing = geometry.packing();
    const auto& declaration = packing.declaration();
    const auto& canonical = declaration.canonical().data();
    const auto& array = output::full_shell::source::FindArray(canonical, "node_ids");
    const auto ids = output::arrays::Decode<std::uint64_t>(array.descriptor, array.bytes);
    Require(ids.size() == forecast.canonical_nodes && ids.capacity() <= 2 * forecast.canonical_nodes,
            "Actual TYPE2 NID decode differs from the complete forecast");
    const auto& domain = *attachments.model().domain();
    auto next = std::make_shared<Data>(attachments);
    next->forecast = forecast;
    next->mains.resize(forecast.mains);
    next->rows.resize(forecast.rows);
    Require(next->mains.capacity() <= 2 * forecast.mains && next->rows.capacity() <= 2 * forecast.rows,
            "Actual TYPE2 retained capacity exceeds its forecast");
    for (std::size_t main = 0; main < forecast.mains; ++main) {
        const auto& source = geometry.data().masters[main];
        Require(main < packing.data().master_rows.size() &&
                    source.declaration_row == packing.data().master_rows[main] &&
                    source.declaration_row < declaration.data().masters.size(),
                "TYPE2 original IRECT ownership differs");
        const auto& element = declaration.data().masters[source.declaration_row];
        Require(element.family == modelio::tied_shell::ElementFamily::Shell &&
                    (element.arity == 3 || element.arity == 4),
                "TYPE2 initializer requires the actual declared shell patch");
        for (unsigned corner = 0; corner < 4; ++corner) {
            const auto working = source.working_nodes[corner];
            Require(working < geometry.data().canonical_nodes.size(), "Invalid TYPE2 working node");
            const auto canonical_node = geometry.data().canonical_nodes[working];
            Require(canonical_node < ids.size(), "Invalid TYPE2 canonical node");
            next->mains[main].nodes[corner] = DomainNode(domain, ids[canonical_node]);
        }
    }
    for (std::size_t row = 0; row < forecast.rows; ++row) {
        const auto original = finalized.data().slaves[row];
        const auto rank = finalized.data().selected_masters[row];
        const auto& before = post.result().slaves().data[row].before;
        const auto& classification = classified.data().slaves[row];
        const auto& attached = attachments.model().rows().data[row];
        Require(original < declaration.data().slave_nodes.size() && rank && rank <= forecast.mains &&
                    classification.original_nsv_row == original && attached.original_nsv_row == original &&
                    attached.ordered_master_rank == rank &&
                    classification.source_node_id == declaration.data().slave_nodes[original].id &&
                    before.source_id == classification.source_node_id && before.irupt == classification.irupt,
                "Finalized TYPE2 compact NSV/IRUPT/IRECT association differs");
        const auto node = DomainNode(domain, before.source_id);
        Require(node == attached.secondary_domain_node, "TYPE2 attachment secondary domain changed");
        for (unsigned corner = 0; corner < 4; ++corner)
            Require(next->mains[rank - 1].nodes[corner] == attached.master_domain_nodes[corner],
                    "TYPE2 original mechanical patch differs from the actual attachment");
        next->rows[row] = {node, static_cast<int>(rank), before.irupt};
    }
    const auto& identity = Interface(controls, post);
    next->interface = {identity.native_id, identity.native_storage_ordinal, 28,
        next->mains.data(), next->mains.size(), next->rows.data(), next->rows.size()};
    return TiedRemovalSource(std::move(next));
}
const vehicle_startup::TiedCinAttachments& TiedRemovalSource::attachments() const noexcept {
    return data_->attachments;
}
tl::util::ConstView<tied_removal::Interface> TiedRemovalSource::interfaces() const noexcept {
    return {&data_->interface, 1};
}
const TiedRemovalForecast& TiedRemovalSource::forecast() const noexcept { return data_->forecast; }
} // namespace crash::cases::vehicle_native_contact
