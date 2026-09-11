#include "Internal.h"

namespace crash::cases::vehicle_startup::post_kinchk_detail {
native_search::KinChkInput Inputs::View(const TiedClassificationData& classified,
        const TiedPostKinChkReceipt& receipt) const {
    return {receipt.native_profile,classified.phase,receipt.source_instance_id,
        static_cast<std::uint32_t>(receipt.original_contact.first_line),
        {slaves.data(),slaves.size()},{classified.interface_decode.data(),classified.interface_decode.size()}};
}
Inputs Pack(const TiedClassificationData& classified, const TiedPostKinChkReceipt& receipt) {
    using output::Require;
    Require(classified.phase == native_search::ClassificationPhase::InterfaceTaggedBeforeKinChk &&
            !classified.slaves.empty() && classified.slaves.size() <= receipt.observed_slaves &&
            classified.cin_count <= classified.slaves.size() &&
            classified.penalty_count == classified.slaves.size()-classified.cin_count &&
            receipt.native_profile == native_search::KinChkProfile::NoWallRbeOrCyclic &&
            receipt.effective_primitive_walls == 0 && receipt.rbe2_roles == 0 &&
            receipt.rbe3_roles == 0 && receipt.cyclic_roles == 0,
            "Post-KINCHK observed phase/count or excluded-role profile differs");
    Inputs out;
    out.slaves.reserve(classified.slaves.size());
    std::size_t cin_count = 0;
    std::uint32_t previous_row = 0;
    bool first = true;
    for (const auto& row : classified.slaves) {
        Require(row.source_node_id && row.source_node_id <= INT32_MAX && row.original_nsv_row < receipt.observed_slaves &&
                (row.irupt == 0 || row.irupt == 1),"Post-KINCHK original slave association differs");
        Require(first || row.original_nsv_row > previous_row,"Post-KINCHK compact NSV order changed");
        first = false;
        previous_row = row.original_nsv_row;
        if (row.irupt == 0) ++cin_count;
        out.slaves.push_back({static_cast<std::uint32_t>(row.source_node_id),row.irupt,row.kinematics});
    }
    Require(cin_count == classified.cin_count,"Post-KINCHK changed the classified CIN/PEN ledger");
    return out;
}
}
