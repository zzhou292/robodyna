#include "Fixture.h"
#include "NativeSolidSupport.h"
namespace crash::cases::vehicle_self_contact::native::main_coefficients::test {
namespace {
struct SolidFixture {
    coated::Inputs input;
    std::vector<std::uint8_t> flags;
    coated::s::Main face;
    SolidFixture() {
        const n::Vector cube[]{{0,0,0},{1,0,0},{1,1,0},{0,1,0},
            {0,0,1},{1,0,1},{1,1,1},{0,1,1},
            {0,0,-1},{1,0,-1},{1,1,-1},{0,1,-1}};
        for (std::size_t i=0;i<12;++i) input.nodes.push_back({1000+i,std::uint32_t(i),cube[i]});
        for (std::size_t i=12;i<24;++i) input.nodes.push_back({1000+i,std::uint32_t(i),{double(i),2,3}});
        const unsigned words[]{0,1,2,3};
        std::copy_n(words,4,face.nodes);
        Add(101,{0,1,2,3,4,5,6,7});
    }
    void Add(std::uint64_t eid,std::array<std::uint32_t,8> nodes,coated::ReaderKind kind=coated::ReaderKind::Hex8) {
        coated::Solid value;
        value.source_id=eid;
        value.part_id=10;
        value.nodes=nodes;
        value.kind=kind;
        input.solids.push_back(value);
        flags.push_back(1);
    }
    detail::SolidSupportQueryIndex Index(std::size_t cap=std::size_t{64}<<20) const {
        return detail::PrepareSolidSupportQueries(input,{flags.data(),flags.size()},cap);
    }
};
void CheckWholeSource(const SolidFixture& fixture) {
    const auto index=fixture.Index();
    const auto result=detail::QuerySolidSupport(fixture.input,index,fixture.face);
    const auto native=NativeSolidSupport(fixture.input,fixture.flags,fixture.face);
    auto expected=native.native_matches;
    std::sort(expected.begin(),expected.end());
    EXPECT_EQ(result.matches,expected);
    if (result.matches.empty()) {
        EXPECT_EQ(result.state,detail::SolidSupportState::Absent);
        EXPECT_EQ(result.first,SIZE_MAX);
        EXPECT_FALSE(native.geometry_defined);
        return;
    }
    if (result.matches.size()>2) {
        EXPECT_EQ(result.state,detail::SolidSupportState::NeedsNativeReaderOrder);
        EXPECT_EQ(result.first,SIZE_MAX);
        EXPECT_EQ(result.second,SIZE_MAX);
        EXPECT_TRUE(native.warning);
        return;
    }
    ASSERT_EQ(result.state,detail::SolidSupportState::Resolved);
    EXPECT_FALSE(native.warning);
    EXPECT_EQ(result.first,native.chosen);
    EXPECT_EQ(native.raw_owner_words[0],int(result.first+1));
    EXPECT_EQ(native.raw_owner_words[1],result.second==SIZE_MAX?0:int(result.second+1));
    EXPECT_EQ(native.effective_count,result.exterior_orientation?1u:2u);
    if (result.exterior_orientation) {
        n::NativeExteriorMainGeometryInput geometry;
        geometry.layout=fixture.face.nodes[2]==fixture.face.nodes[3]?n::ShellLayout::Triangle3:n::ShellLayout::Quad4;
        for (unsigned k=0;k<4;++k) geometry.face[k]=fixture.input.nodes[fixture.face.nodes[k]].native_position;
        for (unsigned k=0;k<8;++k)
            geometry.solid_raw[k]=fixture.input.nodes[fixture.input.solids[result.first].nodes[k]].native_position;
        n::NativeExteriorMainGeometryResult value;
        ASSERT_EQ(n::EvaluateNativeExteriorMainGeometry(geometry,&value),n::CoefficientStatus::Ok);
        EXPECT_TRUE(tl::math::SameScalarBits(value.area,native.area));
        EXPECT_EQ(unsigned(value.reversed),native.inversions);
        for (unsigned k=0;k<4;++k)
            EXPECT_EQ(fixture.face.nodes[value.source_corner[k]],native.final_nodes[k]);
    } else {
        EXPECT_EQ(native.inversions,0u);
        for (unsigned k=0;k<4;++k) EXPECT_EQ(fixture.face.nodes[k],native.final_nodes[k]);
    }
}
}
TEST(MixedSolidSupport, MaximumSourceIdAndEveryFlagPairMatchWholeNative) {
    for (const bool reverse:{false,true}) {
        SolidFixture fixture;
        fixture.Add(303,{8,9,10,11,0,1,2,3});
        if (reverse) std::reverse(fixture.input.solids.begin(),fixture.input.solids.end());
        for (unsigned mask=0;mask<4;++mask) {
            fixture.flags={std::uint8_t(mask&1),std::uint8_t((mask>>1)&1)};
            ASSERT_NO_FATAL_FAILURE(CheckWholeSource(fixture));
            const auto selected=detail::QuerySolidSupport(fixture.input,fixture.Index(),fixture.face);
            if (mask==0 || mask==3) EXPECT_EQ(fixture.input.solids[selected.first].source_id,303u);
            if (mask==3) ASSERT_NE(selected.second,SIZE_MAX);
            else EXPECT_EQ(selected.second,SIZE_MAX);
        }
    }
}
TEST(MixedSolidSupport, ReaderPentaAndCollapsedRaw8KeepEverySourceSlotButCountUniqueMembers) {
    SolidFixture penta;
    penta.input.solids[0].kind=coated::ReaderKind::DeclaredPenta6;
    penta.input.solids[0].nodes={0,1,2,0,4,5,6,4};
    penta.face.nodes[3]=penta.face.nodes[2];
    ASSERT_NO_FATAL_FAILURE(CheckWholeSource(penta));
    const auto native=NativeSolidSupport(penta.input,penta.flags,penta.face);
    EXPECT_EQ(native.native_matches.size(),1u);
    SolidFixture collapsed;
    collapsed.input.solids[0].nodes={0,1,2,3,4,4,5,5};
    ASSERT_NO_FATAL_FAILURE(CheckWholeSource(collapsed));
}
TEST(MixedSolidSupport, MoreThanTwoPreservesAllMatchesAndRejectsUncertifiedFirstPair) {
    SolidFixture fixture;
    fixture.Add(202,{0,1,2,3,4,5,6,7});
    fixture.Add(303,{0,1,2,3,4,5,6,7});
    ASSERT_NO_FATAL_FAILURE(CheckWholeSource(fixture));
    const auto first=NativeSolidSupport(fixture.input,fixture.flags,fixture.face);
    ASSERT_EQ(first.native_matches.size(),3u);
    const auto first_eid=fixture.input.solids[std::size_t(first.raw_owner_words[0]-1)].source_id;
    EXPECT_EQ(fixture.input.solids[first.chosen].source_id,303u);
    std::reverse(fixture.input.solids.begin(),fixture.input.solids.end());
    ASSERT_NO_FATAL_FAILURE(CheckWholeSource(fixture));
    const auto second=NativeSolidSupport(fixture.input,fixture.flags,fixture.face);
    const auto second_eid=fixture.input.solids[std::size_t(second.raw_owner_words[0]-1)].source_id;
    EXPECT_NE(first_eid,second_eid); // Real original first-two incidence dependence.
    const auto selected=detail::QuerySolidSupport(fixture.input,fixture.Index(),fixture.face);
    EXPECT_EQ(selected.state,detail::SolidSupportState::NeedsNativeReaderOrder);
    EXPECT_EQ(selected.matches.size(),3u);
}
TEST(MixedSolidSupport, NoSupportPreservesSeededNativeStorageWithoutInventingAValue) {
    SolidFixture fixture;
    const unsigned absent[]{12,13,14,15};
    std::copy_n(absent,4,fixture.face.nodes);
    ASSERT_NO_FATAL_FAILURE(CheckWholeSource(fixture));
    const auto native=NativeSolidSupport(fixture.input,fixture.flags,fixture.face,{31,47});
    EXPECT_FALSE(native.geometry_defined);
    EXPECT_EQ(native.raw_owner_words[0],31);
    EXPECT_EQ(native.raw_owner_words[1],47);
    EXPECT_EQ(native.area,17.25);
    fixture.input.solids.clear();
    fixture.flags.clear();
    ASSERT_NO_FATAL_FAILURE(CheckWholeSource(fixture));
}
TEST(MixedSolidSupport, IdentityFlagPhaseAndCapacityRejectionsPreserveInputs) {
    SolidFixture fixture;
    const auto need=detail::SolidSupportQueryBytes(fixture.input.nodes.size(),fixture.input.solids.size());
    const auto index=fixture.Index(need);
    EXPECT_THROW(fixture.Index(need-1),detail::Failure);
    EXPECT_EQ(fixture.input.solids[0].source_id,101u);
    auto foreign=fixture.input;
    EXPECT_THROW(detail::QuerySolidSupport(foreign,index,fixture.face),std::exception);
    fixture.flags[0]=2;
    EXPECT_THROW(fixture.Index(),std::exception);
    fixture.flags[0]=1;
    fixture.Add(101,{8,9,10,11,0,1,2,3});
    EXPECT_THROW(fixture.Index(),std::exception);
    fixture.input.solids.pop_back();
    fixture.flags.pop_back();
    fixture.input.solids[0].phase=static_cast<coated::PacketPhase>(99);
    EXPECT_THROW(fixture.Index(),std::exception);
    fixture.input.solids[0].phase=coated::PacketPhase::ReaderBeforeInitia;
    fixture.input.solids[0].source_id=UINT64_C(0x80000000);
    EXPECT_THROW(fixture.Index(),std::exception);
    EXPECT_THROW(detail::SolidSupportQueryBytes(1,16385),detail::Failure);
}
}
