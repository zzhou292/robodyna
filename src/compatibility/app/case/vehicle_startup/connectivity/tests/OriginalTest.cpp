#include "../VehicleConnectivity.h"
#include "../../physical_attachments/tests/Support.h"
#include <map>

namespace crash::cases::vehicle_startup::connectivity::test {
const VehicleConnectivity& Actual() {
    static const auto value = [] {
        const auto& input = physical_attachments::test::Actual();
        const auto forecast = VehicleConnectivity::Preflight(input);
        std::cout << "Connectivity forecast before full graph: source=" << forecast.retained_source_bound
                  << " graph=" << forecast.owned_bytes << " scratch=" << forecast.component_scratch_bytes
                  << " extra=" << forecast.extra_bytes << " total=" << forecast.total_bytes << std::endl;
        return VehicleConnectivity::Prepare(input);
    }();
    return value;
}
TEST(VehicleConnectivityOriginal, CompleteTypedSourceRowsAndPotentialTransferReport) {
    const auto& value=Actual();
    const auto& data=value.data();
    const auto& physical=value.source().physical();
    const auto& domain=physical.source_domain().domain();
    const auto& counts=data.counts;
    EXPECT_EQ(counts.nodes,372435u);
    EXPECT_EQ(counts.relations,371413u);
    const auto count=[&](Kind k) { return counts.by_kind[static_cast<std::size_t>(k)]; };
    EXPECT_EQ(count(Kind::Qeph)+count(Kind::T3)+count(Kind::Qbat),349645u);
    EXPECT_EQ(count(Kind::Qbat),4250u);
    EXPECT_EQ(count(Kind::Solid18),908u); EXPECT_EQ(count(Kind::Solid24),1309u); EXPECT_EQ(count(Kind::Solid6z),195u);
    EXPECT_EQ(count(Kind::Type13),4442u); EXPECT_EQ(count(Kind::Type25),2828u);
    EXPECT_EQ(count(Kind::PartRoot),20u); EXPECT_EQ(count(Kind::PlainGroup),753u);
    EXPECT_EQ(count(Kind::Cin),11165u); EXPECT_EQ(count(Kind::PointMass),148u);
    EXPECT_EQ(counts.rigid_skin_parents,5102u);
    std::map<std::uint64_t,std::size_t> element_sizes,transfer_sizes;
    for (std::size_t node=0;node<domain.node_count();++node) {
        ASSERT_NE(domain.Find(data.element_label[node]),SIZE_MAX);
        ASSERT_NE(domain.Find(data.transfer_label[node]),SIZE_MAX);
        EXPECT_LE(data.element_label[node],domain.nodes()[node].source_id);
        EXPECT_LE(data.transfer_label[node],data.element_label[node]);
        ++element_sizes[data.element_label[node]]; ++transfer_sizes[data.transfer_label[node]];
    }
    EXPECT_EQ(counts.element_components,element_sizes.size());
    EXPECT_EQ(counts.transfer_components,transfer_sizes.size());
    std::size_t triangles=0,beam_rows=0;
    for (const auto& row : data.relations) {
        ASSERT_GT(row.slot_count,0u);
        const auto first=data.slots[row.slot_offset];
        for (std::size_t slot=row.slot_offset;slot<row.slot_offset+row.slot_count;++slot) {
            ASSERT_LT(data.slots[slot],domain.node_count());
            if (row.role==Role::Constitutive) EXPECT_EQ(data.element_label[first],data.element_label[data.slots[slot]]);
            if (row.role!=Role::CoefficientOnly) EXPECT_EQ(data.transfer_label[first],data.transfer_label[data.slots[slot]]);
        }
        if (row.kind==Kind::Cin) {
            const auto& original=value.source().attachments().model().rows().data[row.source_row];
            EXPECT_EQ(row.source_id,original.master_source.element_id);
            EXPECT_EQ(row.part_id,original.master_source.part_id);
            EXPECT_EQ(data.slots[row.slot_offset],original.secondary_domain_node);
            for (unsigned slot=0;slot<4;++slot) EXPECT_EQ(data.slots[row.slot_offset+1+slot],original.master_domain_nodes[slot]);
            triangles+=data.slots[row.slot_offset+3]==data.slots[row.slot_offset+4];
        } else if (row.kind==Kind::Type13) {
            const auto records=physical.coefficients().type13()->records();
            ASSERT_EQ(row.slot_count,2u);
            EXPECT_EQ(data.slots[row.slot_offset],records[2*row.source_row].value.global_node);
            EXPECT_EQ(data.slots[row.slot_offset+1],records[2*row.source_row+1].value.global_node);
            ++beam_rows;
        }
    }
    EXPECT_EQ(triangles,125u); EXPECT_EQ(beam_rows,4442u);
    EXPECT_EQ(value.required_joints(),Obligation::Pending);
    EXPECT_EQ(value.omitted_source_and_auxiliary_policy(),Obligation::Pending);
    EXPECT_EQ(value.current_cin_activity_and_release(),Obligation::Pending);
    const auto report=ReportJson(value);
    EXPECT_LE(report.size(),Limits{}.report_bytes);
    RecordProperty("element_components",std::to_string(counts.element_components));
    RecordProperty("potential_transfer_components",std::to_string(counts.transfer_components));
    RecordProperty("source_relations",std::to_string(counts.relations));
    RecordProperty("ordered_support_slots",std::to_string(counts.slots));
    RecordProperty("graph_owned_bytes",std::to_string(value.forecast().owned_bytes));
    RecordProperty("extra_phase_max_bytes",std::to_string(value.forecast().extra_bytes));
    RecordProperty("inclusive_total_bytes",std::to_string(value.forecast().total_bytes));
    RecordProperty("report_bytes",std::to_string(report.size()));
    RecordProperty("report_sha256",output::Sha256(report));
    if (const auto* path=std::getenv("ROBO_DYNA_CONNECTIVITY_REPORT"); path && *path) output::WriteBytes(path,report);
}
TEST(VehicleConnectivityOriginal, ExactCapsSourceLifetimeAndLateBudgetRetry) {
    const auto& value=Actual();
    const auto& source=value.source();
    auto limits=Limits{};
    limits.total_bytes=value.forecast().total_bytes-1;
    const auto first=value.data().transfer_label.front(),last=value.data().transfer_label.back();
    EXPECT_THROW(VehicleConnectivity::Prepare(source,limits),std::runtime_error);
    EXPECT_EQ(value.data().transfer_label.front(),first); EXPECT_EQ(value.data().transfer_label.back(),last);
    ++limits.total_bytes;
    EXPECT_EQ(VehicleConnectivity::Preflight(source,limits).total_bytes,value.forecast().total_bytes);
    limits.extra_bytes=value.forecast().extra_bytes-1;
    EXPECT_THROW(VehicleConnectivity::Preflight(source,limits),std::runtime_error);
    ++limits.extra_bytes;
    EXPECT_EQ(VehicleConnectivity::Preflight(source,limits).extra_bytes,value.forecast().extra_bytes);
    auto copy=value;
    EXPECT_EQ(&copy.data(),&value.data());
    EXPECT_EQ(&copy.source().physical().coefficients(),&source.physical().coefficients());
}
} // namespace crash::cases::vehicle_startup::connectivity::test
