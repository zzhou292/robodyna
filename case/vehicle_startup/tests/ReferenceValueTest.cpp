#include "../ReferenceStorage.h"
#include "ReferenceComparison.h"
#include "lib_src/elements/qeph/QephStartup.h"
#include "lib_src/elements/t3/T3Startup.h"
#include <limits>

namespace crash::cases::vehicle_startup::test {
namespace assembly=modelio::assembly;
template<std::size_t N> auto Packet(const assembly::SourceReferenceNode (&nodes)[N]) {
    assembly::Material m;m.density_kg_m3=7800;m.young_pa=210e9;m.poisson_ratio=.3;
    assembly::Section s;s.thickness_m.fill(.0008);
    using Input=std::conditional_t<N==4,tl::fea::qeph::ReferenceInput,tl::fea::t3::ReferenceInput>;
    return assembly::PackShellReference<Input>(nodes,m,s);
}
TEST(VehicleReferenceValue, PackingPreservesSignedZeroAndFamilyNodeWidth) {
    RecordProperty("sizeof_qeph_reference",std::to_string(sizeof(tl::fea::qeph::ReferenceData)));
    RecordProperty("sizeof_t3_reference",std::to_string(sizeof(tl::fea::t3::ReferenceData)));
    RecordProperty("sizeof_reference_row",std::to_string(sizeof(ReferenceRow)));
    assembly::SourceReferenceNode q[4]={{91,{-0.,0,0}},{19,{.04,0,0}},{71,{.04,.03,.0001}},{29,{0,.03,0}}};
    const auto packed=Packet(q);Same(packed.position[0].x,-0.);EXPECT_EQ(packed.node_ids[3],29);
    tl::fea::qeph::ReferenceData native;
    ASSERT_EQ(tl::fea::qeph::InitializeReference(packed,native),tl::fea::qeph::Status::kSuccess);
    detail::ReferenceStorage result;detail::Append(result,{},packed);
    ASSERT_EQ(result.qeph.size(),1);Same(native,result.qeph.front());
    q[3].source_id=std::uint64_t(UINT32_MAX)+1;
    EXPECT_THROW(Packet(q),std::runtime_error);
    assembly::SourceReferenceNode t[3]={q[0],q[1],q[3]};
    EXPECT_EQ(Packet(t).node_ids[2],std::uint64_t(UINT32_MAX)+1);
}
TEST(VehicleReferenceValue, OrderedNativeFailuresRemainVisibleWithoutStaleReferences) {
    assembly::SourceReferenceNode q[4]={{91,{0,0,0}},{19,{.04,0,0}},{71,{.04,.03,0}},{29,{0,.03,0}}};
    assembly::SourceReferenceNode t[3]={q[0],q[1],q[3]};
    detail::ReferenceStorage result;ReferenceRow row;row.element_id=99;row.part_id=8;
    detail::Append(result,row,Packet(q));row.element_id=77;detail::AppendUnresolved(result,row);
    auto bad=Packet(t);bad.position[2]=bad.position[1];row.element_id=500;
    detail::Append(result,row,bad);row.element_id=1;auto badq=Packet(q);badq.density=std::numeric_limits<double>::quiet_NaN();
    detail::Append(result,row,badq);row.element_id=22;detail::Append(result,row,Packet(t));
    ASSERT_EQ(result.rows.size(),5);EXPECT_EQ(result.first_error,2);EXPECT_EQ(result.rows[result.first_error].element_id,500);
    EXPECT_EQ(result.rows[2].status,ReferenceStatus::UnsupportedGeometry);
    EXPECT_EQ(result.rows[3].status,ReferenceStatus::InvalidInput);
    EXPECT_EQ(result.rows[1].family,ReferenceFamily::None);
    for(auto i:{1,2,3})EXPECT_EQ(result.rows[i].reference_index,SIZE_MAX);
    EXPECT_EQ(result.qeph.size(),1);EXPECT_EQ(result.t3.size(),1);EXPECT_EQ(result.counts.rejected,2);
    EXPECT_EQ(result.counts.unresolved,1);EXPECT_EQ(result.counts.attempted,4);EXPECT_EQ(result.counts.succeeded,2);
    tl::fea::t3::ReferenceData native;
    ASSERT_EQ(tl::fea::t3::InitializeReference(Packet(t),native),tl::fea::t3::Status::kSuccess);Same(result.t3[0],native);
}
TEST(VehicleReferenceValue, LatePackingExceptionLeavesPublishedInputAndRetryExact) {
    assembly::SourceReferenceNode q[4]={{91,{0,0,0}},{19,{.04,0,0}},{71,{.04,.03,0}},{29,{0,.03,0}}};
    const auto good=Packet(q);auto visible=good;
    q[3].source_id=std::uint64_t(UINT32_MAX)+1;
    EXPECT_THROW(visible=Packet(q),std::runtime_error);SameInput(visible,good);
    q[3].source_id=29;visible=Packet(q);SameInput(visible,good);
}
} // namespace crash::cases::vehicle_startup::test
