#include "../../VehicleShellBinding.h"
#include "../../tests/ReferenceComparison.h"
#include "modelio/vehicle_sections/tests/RigidSourceSupport.h"
#include <cmath>
#include <limits>

namespace crash::cases::vehicle_startup::test {
namespace source_test=modelio::vehicle::test;
namespace {
const VehicleShellReferences& References() {
    static const auto refs=VehicleShellReferences::Prepare(source_test::RigidResolution());return refs;
}
const VehicleShellBinding& Prepared() {
    static const auto binding=VehicleShellBinding::Prepare(References());return binding;
}
template<class T> auto Decode(const char* name) {
    const auto& a=modelio::vehicle::source::FindArray(source_test::Canonical().data(),name);
    return output::arrays::Decode<T>(a.descriptor,a.bytes);
}
std::array<double,4> Values(const tl::fea::ShellBindingMass& value) {
    return {value.mass,value.isotropic_inertia,value.physical_inertia,value.added_inertia};
}
}
TEST(VehicleShellBindingOriginal, CompleteSourceMappingNativeReferenceParityAndRigidRoles) {
    const auto& complete=Prepared();const auto& refs=complete.references();const auto& binding=complete.shells();
    ASSERT_EQ(refs.rows().size(),349645);EXPECT_EQ(binding.node_count(),359785);
    EXPECT_EQ(binding.qeph_count(),324094);EXPECT_EQ(binding.t3_count(),21301);EXPECT_EQ(binding.qbat_count(),4250);
    std::size_t rigid=0;
    for(std::size_t e=0;e<refs.rows().size();++e) {
        const auto& row=refs.rows()[e];const auto* mapping=refs.resolution()->native_mapping(e);
        ASSERT_NE(mapping,nullptr);const auto i=mapping->family_index;
        if(const auto* q=refs.qeph(e)) {EXPECT_EQ(binding.qeph_source_id(i),row.element_id);Same(binding.qeph_reference(i),*q);}
        else if(const auto* t=refs.t3(e)) {EXPECT_EQ(binding.t3_source_id(i),row.element_id);Same(binding.t3_reference(i),*t);}
        else {
            const auto* b=refs.qbat(e);ASSERT_NE(b,nullptr);EXPECT_EQ(binding.qbat_source_id(i),row.element_id);
            Same(binding.qbat_reference(i).quadrilateral(),b->quadrilateral());
            Same(binding.qbat_reference(i).input().initial_a11_pa,b->input().initial_a11_pa);
        }
        if(row.role==modelio::vehicle::SourceShellRole::OriginalRigidPart) {
            ++rigid;EXPECT_LT(row.rigid_root_index,20);
        }
    }
    EXPECT_EQ(rigid,5102);
    const auto ids=Decode<std::uint64_t>("node_ids");const auto positions=Decode<double>("node_positions");
    for(std::size_t n=0;n<binding.node_count();++n) {
        const auto canonical=refs.source().canonical_nodes().at(n);const auto& node=binding.active_nodes()[n];
        EXPECT_EQ(node.source_id,ids.at(canonical));
        Same(node.position,tl::math::Vec3{positions.at(3*canonical),positions.at(3*canonical+1),positions.at(3*canonical+2)});
    }
    RecordProperty("startup_reservation_bytes",std::to_string(complete.forecast().total_bytes));
    RecordProperty("native_owned_bytes",std::to_string(binding.host_bytes()));
    RecordProperty("native_scratch_bytes",std::to_string(binding.startup_scratch_bytes()));
    RecordProperty("shell_mass_kg",std::to_string(binding.totals().mass));
}
TEST(VehicleShellBindingOriginal, EverySourceParentContributesOnceIncludingCoincidentLayers) {
    const auto& refs=References();const auto& binding=Prepared().shells();
    std::vector<std::array<long double,4>> exact(binding.node_count());
    std::vector<std::size_t> occurrences(binding.node_count());
    auto add=[&](const auto& reference,const auto& nodes) {
        for(std::size_t k=0;k<nodes.size();++k) {
            const auto n=nodes[k];++occurrences[n];
            exact[n][0]+=reference.nodal_mass[k];exact[n][1]+=reference.isotropic_inertia[k];
            exact[n][2]+=reference.physical_inertia[k];exact[n][3]+=reference.added_inertia[k];
        }
    };
    for(std::size_t e=0;e<refs.rows().size();++e) {
        const auto i=refs.resolution()->native_mapping(e)->family_index;
        if(const auto* q=refs.qeph(e)) add(*q,binding.qeph_nodes(i));
        else if(const auto* t=refs.t3(e)) add(*t,binding.t3_nodes(i));
        else add(refs.qbat(e)->quadrilateral(),binding.qbat_nodes(i));
    }
    std::array<long double,4> total{};std::size_t count=0;
    for(std::size_t n=0;n<binding.node_count();++n) {
        ASSERT_GT(occurrences[n],0);count+=occurrences[n];const auto actual=Values(binding.nodes()[n].native);
        for(std::size_t c=0;c<4;++c) {
            total[c]+=exact[n][c];
            const auto bound=4*occurrences[n]*std::numeric_limits<double>::epsilon()*exact[n][c];
            EXPECT_LE(std::abs(static_cast<long double>(actual[c])-exact[n][c]),bound);
        }
    }
    EXPECT_EQ(count,4*(324094+4250)+3*21301);
    const auto global=Values(binding.totals());
    for(std::size_t c=0;c<4;++c)
        EXPECT_LE(std::abs(static_cast<long double>(global[c])-total[c]),
            4*count*std::numeric_limits<double>::epsilon()*total[c]);
}
TEST(VehicleShellBindingOriginal, IncompleteProfileAndAllocationLimitsRejectWhilePublishedBindingSurvives) {
    const auto& original=Prepared();const auto* nodes=original.shells().active_nodes().data();
    auto limits=VehicleShellBindingLimits{};const auto bytes=original.forecast().total_bytes;
    limits.host_bytes=bytes-1;EXPECT_THROW(VehicleShellBinding::Prepare(References(),limits),std::runtime_error);
    limits.host_bytes=bytes;EXPECT_EQ(ForecastShellBinding(References(),limits).total_bytes,bytes);
    limits.native.max_nodes=359784;EXPECT_THROW(VehicleShellBinding::Prepare(References(),limits),std::runtime_error);
    const auto incomplete=VehicleShellReferences::Prepare(source_test::MidlayerResolution());
    EXPECT_THROW(VehicleShellBinding::Prepare(incomplete),std::runtime_error);
    const auto retry=VehicleShellBinding::Prepare(References());
    EXPECT_TRUE(retry.shells().inventory()==original.shells().inventory());
    EXPECT_EQ(original.shells().active_nodes().data(),nodes);
    Same(retry.shells().totals().mass,original.shells().totals().mass);
    const auto copy=[] {const auto value=Prepared();return VehicleShellBinding(value);}();
    EXPECT_EQ(copy.shells().active_nodes().data(),nodes);
}
}
