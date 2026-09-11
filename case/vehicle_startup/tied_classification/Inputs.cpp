#include "Internal.h"
#include <algorithm>
#include <charconv>

namespace crash::cases::vehicle_startup::tied_classification_detail {
native_search::ClassificationInput Inputs::View() {
    interface.slaves = {slaves.data(),slaves.size()};
    interface.masters = {masters.data(),masters.size()};
    native_search::ClassificationInput result;
    result.context = {source_instance,{nodes.data(),nodes.size()}};
    result.interfaces = {&interface,1};
    return result;
}
Inputs Pack(const tied::source::CanonicalData& canonical, const tied::Data& declaration,
        const tied::SearchGeometryData& geometry, const native_search::FinalizationMaps& finalized,
        const TiedFinalizationReceipt& receipt) {
    using output::Require;
    Require(receipt.phase == TiedFinalizationPhase::FreshPhysicalCoefficientsBeforeIniend &&
            receipt.is1 == 2 && receipt.level == 28 && receipt.type2_count == 1 &&
            receipt.type2_ordinal == 1 && receipt.prior_connection_count == 0 &&
            receipt.multiple_connection_count == 0 && receipt.ignore == 2 && receipt.projection == 1 &&
            receipt.search_distance == 0 && receipt.contact_source == declaration.contact_source &&
            receipt.unique_original_slaves == declaration.slave_nodes.size(),
            "Classification finalization caller phase differs");
    Require(declaration.contact_source < declaration.sources.size() &&
            geometry.canonical_nodes.size() == geometry.working_positions.size() &&
            geometry.secondary_working_nodes.size() == declaration.slave_nodes.size() &&
            finalized.dispositions.size() == declaration.slave_nodes.size(),
            "Classification source/finalized extent differs");
    Inputs out;
    const auto& hash = canonical.inputs.source_member.sha256;
    output::arrays::CheckHash(hash);
    const auto parsed = std::from_chars(hash.data(),hash.data()+16,out.source_instance,16);
    Require(parsed.ec == std::errc{} && parsed.ptr == hash.data()+16 && out.source_instance,
            "Invalid source instance association");
    const auto& array = tied::source::FindArray(canonical,"node_ids");
    const auto source_ids = output::arrays::Decode<tied::SourceId>(array.descriptor,array.bytes);
    out.nodes.reserve(geometry.canonical_nodes.size());
    for (const auto row : geometry.canonical_nodes) {
        Require(row < source_ids.size() && source_ids[row] && source_ids[row] <= UINT32_MAX,
                "Classification original node association changed");
        out.nodes.push_back({static_cast<std::uint32_t>(source_ids[row]),{}});
    }
    out.slaves.reserve(finalized.slaves.size());
    for (const auto row : finalized.slaves) {
        Require(row < declaration.slave_nodes.size() &&
                finalized.dispositions[row] == native_search::FinalizationDisposition::Kept,
                "Classification includes a removed original slave");
        const auto working = geometry.secondary_working_nodes[row];
        Require(working < out.nodes.size() && out.nodes[working].source_id == declaration.slave_nodes[row].id,
                "Classification original NSV identity changed");
        out.slaves.push_back(working);
    }
    out.masters.reserve(finalized.main_nodes.size());
    for (const auto row : finalized.main_nodes) {
        Require(row < declaration.master_nodes.size(), "Classification MSR rank is invalid");
        const auto canonical_node = declaration.master_nodes[row];
        const auto found = std::lower_bound(geometry.canonical_nodes.begin(),geometry.canonical_nodes.end(),canonical_node);
        Require(found != geometry.canonical_nodes.end() && *found == canonical_node,
                "Classification original MSR association changed");
        out.masters.push_back(static_cast<std::uint32_t>(found-geometry.canonical_nodes.begin()));
    }
    // Original source line is a caller association, not a guessed generated
    // Radioss interface ID. The complete source block remains in the handle.
    const auto line = declaration.sources[declaration.contact_source].block.first_line;
    Require(line && line <= UINT32_MAX, "Classification source interface association overflow");
    out.interface.source_id = static_cast<std::uint32_t>(line);
    out.interface.native_type = 2;
    out.interface.level = 28;
    return out;
}
TiedClassificationData Observed(const native_search::ClassificationResult& result,
        const tied::Data& declaration, const native_search::FinalizationMaps& finalized) {
    using output::Require;
    Require(result.phase() == native_search::ClassificationPhase::InterfaceTaggedBeforeKinChk &&
            result.slave_nodes().count == finalized.slaves.size() && result.irupt().count == finalized.slaves.size() &&
            result.interface_decode().count == 8192,
            "Classification result phase or complete extent differs");
    TiedClassificationData out;
    out.slaves.reserve(finalized.slaves.size());
    for (std::size_t s = 0; s < finalized.slaves.size(); ++s) {
        const auto original = finalized.slaves[s];
        const auto node = result.slave_nodes().data[s];
        const auto irupt = result.irupt().data[s];
        Require(original < declaration.slave_nodes.size() && node < result.nodes().count &&
                declaration.slave_nodes[original].id == result.nodes().data[node].source_id &&
                (irupt == 0 || irupt == 1), "Classification result original slave association differs");
        out.slaves.push_back({declaration.slave_nodes[original].id,original,irupt,result.nodes().data[node].kinematics});
        if (irupt == 0) ++out.cin_count;
        else ++out.penalty_count;
    }
    std::copy_n(result.interface_decode().data,out.interface_decode.size(),out.interface_decode.begin());
    out.kinset_warning_count = result.native_kinset_warnings();
    out.penalty_warning_count = result.native_penalty_warnings();
    out.phase = result.phase();
    out.owned_payload_bytes = sizeof(out) + out.slaves.capacity() * sizeof(ClassifiedTiedSlave);
    return out;
}
}
