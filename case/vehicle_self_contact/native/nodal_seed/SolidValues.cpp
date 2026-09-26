#include "SolidSlots.h"
#include "MaterialSlots.h"
#include <algorithm>

namespace crash::cases::vehicle_self_contact::native::nodal_seed::detail {
namespace {
template<class Parent>
void Append(const modelio::solid_source::Row& source,
    const modelio::solid_source::Part& part, const Parent& parent,
    const tl::fea::NodalNodeDomain& domain, std::size_t source_index,
    ContributorKind kind, unsigned slots, double volume_si, double bulk_si,
    const n::coefficient_detail::UnitFactors& factors, Packed& output) {
    Require(parent.reference.input().source_material_id == part.material_id &&
        parent.reference.input().source_section_id == part.section_id &&
        part.id == source.part_id, "Contact seed solid property/material association differs");
    Contributor row;
    row.kind = kind;
    row.channel = Channel::Volume;
    row.original_id = source.element_id;
    // The declared V5 native H8/PENTA reader packet retains original solid EIDs.
    // This is not an assertion about omitted full-original converter families.
    row.native_id = source.element_id;
    row.part_id = source.part_id;
    row.material_id = part.material_id;
    row.source_index = source_index;
    row.nodes = PostInitiaSolidSlots(source, parent, domain, slots);
    row.slots = 8;

    n::NativeSolidNodalInput input;
    input.kind = slots == 6 ? n::SolidNodalKind::Penta6 : n::SolidNodalKind::Hex8;
    input.volume = volume_si / factors.volume;
    input.bulk = bulk_si / factors.pressure;
    // MATING's virgin no-initial-stress branch, retained by the selected V5
    // solid source profiles. CheckModel authenticates those prepared handles.
    input.fill = 1.;
    n::NativeSolidNodalShares shares;
    Require(n::EvaluateNativeSolidNodalShares(input, &shares) == n::CoefficientStatus::Ok,
        "Native solid contact contribution rejected prepared source operands");
    Require(shares.defined_raw_slot_mask == (slots == 6 ? 0x77 : 0xff),
        "Native solid contact contribution write mask changed");
    output.contributors.push_back(row);
    for (unsigned slot = 0; slot < 8; ++slot) {
        // LECTUR initializes the actual VNS/BNS arrays to +0 before INITIA.
        // SBULK3 leaves Penta raw slots4/8 untouched; retain those occurrences.
        n::NativeVolumeOccurrence value;
        value.node = row.nodes[slot];
        if (shares.defined_raw_slot_mask & (1u << slot)) {
            value.volume = shares.volume_share;
            value.bulk_volume = shares.bulk_volume_share;
        }
        output.volumes.push_back(value);
    }
}
}

void PackSolids(const PhysicalModel& model, n::UnitScale units, Packed& output) {
    const auto factors = Factors(units);
    // HM_READ_MAT42 PARMAT1=GS; generic HM_READ_MAT transfers it to PM32.
    // For this immutable alpha2/no-Prony source profile, UPDMAT GammaInf=1
    // and LAW42_UPD does not replace PM32. Parameters.bulk_pa is PM100,
    // not the SBULK3 input. GS=2*mu is distinct from that physical bulk.
    const auto& source = model.source_domain().source().solid_source().data();
    const auto& physical = model.solids();
    const auto& domain = model.source_domain().domain();
    std::vector<std::size_t> order;
    order.reserve(source.rows.size());
    for (std::size_t i = 0; i < source.rows.size(); ++i) order.push_back(i);
    std::sort(order.begin(), order.end(), [&](auto a, auto b) {
        return source.rows[a].element_id < source.rows[b].element_id;
    });
    std::uint64_t previous = 0;
    for (const auto index : order) {
        const auto& row = source.rows[index];
        Require(row.element_id > previous && row.part_index < source.parts.size(),
            "Contact seed solid source ID/order differs");
        previous = row.element_id;
        const auto& part = source.parts[row.part_index];
        const auto i = row.reference_index;
        using F = modelio::solid_source::Family;
        switch (row.family) {
        case F::Solid18: {
            Require(i < physical.solid18().size(), "Missing physical LAW36 solid");
            const auto& parent = physical.solid18()[i];
            Append(row, part, parent, domain, index, ContributorKind::Solid18Law36, 8,
                parent.reference.geometry().center_volume_m3, part.law36.bulk_pa, factors, output);
            break;
        }
        case F::Solid24: {
            Require(i < physical.solid24().size(), "Missing physical HEPH solid");
            const auto& parent = physical.solid24()[i];
            Append(row, part, parent, domain, index, ContributorKind::HephLaw42, 8,
                parent.reference.geometry().volume_m3, Law42ContactPm32Pa(part.law42), factors, output);
            break;
        }
        case F::Solid6z: {
            Require(i < physical.solid6z().size(), "Missing physical Penta solid");
            const auto& parent = physical.solid6z()[i];
            Append(row, part, parent, domain, index, ContributorKind::PentaLaw42, 6,
                parent.reference.geometry().volume_m3, Law42ContactPm32Pa(part.law42), factors, output);
            break;
        }
        case F::Solid18Law44: {
            Require(i < physical.solid18_law44().size(), "Missing physical LAW44 solid");
            const auto& parent = physical.solid18_law44()[i];
            Append(row, part, parent, domain, index, ContributorKind::Solid18Law44, 8,
                parent.reference.geometry().center_volume_m3, part.law44.bulk_pa, factors, output);
            break;
        }
        case F::Solid18Law90: {
            Require(i < physical.solid18_law90().size(), "Missing physical LAW90 solid");
            const auto& parent = physical.solid18_law90()[i];
            Append(row, part, parent, domain, index, ContributorKind::Solid18Law90, 8,
                parent.reference.geometry().center_volume_m3, part.law90.updated().bulk_pa, factors, output);
            break;
        }
        default:
            Require(false, "Unsupported contact seed solid family");
        }
    }
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_seed::detail
