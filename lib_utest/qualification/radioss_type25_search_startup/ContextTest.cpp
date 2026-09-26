#include "Assertions.h"
#include <type_traits>
namespace type25_search_startup_test {
namespace {
struct Storage {
  tl::util::HostArena out,scratch;s::Snapshot result;
  explicit Storage(const s::Input& in) {
    const auto f=s::Preflight(in);if(f.status!=s::Status::Ok||!out.Initialize(f.output_bytes)||!scratch.Initialize(f.scratch_bytes))
      throw std::runtime_error("Context fixture allocation rejected");
  }
  std::vector<unsigned char> Bytes() const {const auto* p=static_cast<const unsigned char*>(out.data());return {p,p+out.bytes()};}
};
s::Input Rigid(const Fixture& f,const std::uint64_t* id) {
  auto in=f.Input();in.contributors.rigid_bodies=1;in.contributors.native_auxiliary_nodes=1;
  in.auxiliary_rigid_primary_ids=id;in.auxiliary_rigid_primary_count=1;return in;
}
}
TEST(Type25SearchContext, LegacyForecastAndEveryNativeValueRemainUnchanged) {
  Fixture f;const auto in=f.Input();
  const auto a=s::Preflight(in.mesh.node_count,in.mesh.primary_count,in.secondary_count),b=s::Preflight(in);
  EXPECT_EQ(a.output_bytes,b.output_bytes);EXPECT_EQ(a.scratch_bytes,b.scratch_bytes);EXPECT_EQ(a.removal_capacity,b.removal_capacity);
  const Built old(f);Same(old.view,Oracle(in));EXPECT_EQ(old.view.native_model_nodes,in.mesh.node_count);
}
TEST(Type25SearchContext, RealEighteenPlusOnePopulationUsesSeparateAuxiliaryIdentity) {
  Fixture f(old::Grid(2,5));ASSERT_EQ(f.mesh.ids.size(),18u);const std::uint64_t primary=19;
  auto in=Rigid(f,&primary);Storage memory(in);
  EXPECT_EQ(s::Build(in,{},memory.out,memory.scratch,&memory.result).status,s::Status::UnsupportedProfile);
  ASSERT_EQ(s::BuildRigidOnly(in,{},memory.out,memory.scratch,&memory.result).status,s::Status::Ok);
  Same(memory.result,Oracle(in));EXPECT_EQ(memory.result.native_model_nodes,19u);
  EXPECT_EQ(in.mesh.node_count,18u);EXPECT_EQ(memory.result.secondary_count,18u);
  EXPECT_EQ(Bits(memory.result.multiplier),Bits(OracleMultiplier(19)));
}
TEST(Type25SearchContext, AuxiliaryIdentityMismatchDuplicateAndPhysicalCollisionPreservePublication) {
  Fixture f;std::uint64_t primary=100;auto in=Rigid(f,&primary);Storage memory(in);
  ASSERT_EQ(s::BuildRigidOnly(in,{},memory.out,memory.scratch,&memory.result).status,s::Status::Ok);
  const auto saved=memory.Bytes();const auto snapshot=memory.result;
  for(auto wrong:{std::uint64_t(0),f.mesh.ids[0],std::uint64_t(INT32_MAX)+1}) {
    primary=wrong;
    EXPECT_EQ(s::BuildRigidOnly(in,{},memory.out,memory.scratch,&memory.result).status,s::Status::InvalidInput);
    EXPECT_EQ(memory.Bytes(),saved);EXPECT_EQ(memory.result.primary_extent,snapshot.primary_extent);
    EXPECT_EQ(memory.result.native_model_nodes,snapshot.native_model_nodes);
  }
  primary=100;auto mismatch=in;mismatch.contributors.native_auxiliary_nodes=0;
  EXPECT_EQ(s::BuildRigidOnly(mismatch,{},memory.out,memory.scratch,&memory.result).status,s::Status::InvalidInput);
  std::uint64_t duplicate[]{100,100};in.contributors.rigid_bodies=2;in.contributors.native_auxiliary_nodes=2;
  in.auxiliary_rigid_primary_ids=duplicate;in.auxiliary_rigid_primary_count=2;Storage repeated(in);
  EXPECT_EQ(s::BuildRigidOnly(in,{},repeated.out,repeated.scratch,&repeated.result).status,s::Status::InvalidInput);
  duplicate[1]=101;
  ASSERT_EQ(s::BuildRigidOnly(in,{},repeated.out,repeated.scratch,&repeated.result).status,s::Status::Ok);
}
TEST(Type25SearchContext, TiedGeometryIsTypedPendingAndCannotUseEitherCompleteEntry) {
  static_assert(!std::is_convertible_v<s::GeometricSnapshot,s::Snapshot>);
  Fixture f;auto in=f.Input();in.contributors.tied_interfaces=1;in.contributors.cin_links=4;
  Storage memory(in);s::GeometricSnapshot pending;
  EXPECT_EQ(s::Build(in,{},memory.out,memory.scratch,&memory.result).status,s::Status::UnsupportedProfile);
  EXPECT_EQ(s::BuildRigidOnly(in,{},memory.out,memory.scratch,&memory.result).status,s::Status::UnsupportedProfile);
  ASSERT_EQ(s::BuildGeometricBeforeTied(in,{},memory.out,memory.scratch,&pending).status,s::Status::Ok);
  Same(pending.geometry,Oracle(in));EXPECT_EQ(pending.contributors.tied_interfaces,1u);EXPECT_EQ(pending.contributors.cin_links,4u);
  EXPECT_EQ(pending.geometry.native_model_nodes,in.mesh.node_count);
}
TEST(Type25SearchContext, ModelCountCapsAreSeparateFromGeometryAndRejectBeforeBorrowedReads) {
  Fixture f;const std::uint64_t primary=100;const auto in=Rigid(f,&primary);Storage memory(in);
  s::Limits cap;cap.max_nodes=in.mesh.node_count;cap.max_native_model_nodes=in.mesh.node_count+1;
  ASSERT_EQ(s::BuildRigidOnly(in,cap,memory.out,memory.scratch,&memory.result).status,s::Status::Ok);
  const auto saved=memory.Bytes();--cap.max_native_model_nodes;
  EXPECT_EQ(s::BuildRigidOnly(in,cap,memory.out,memory.scratch,&memory.result).status,s::Status::ResourceLimit);
  EXPECT_EQ(memory.Bytes(),saved);
  auto wrong=in;wrong.contributors.native_auxiliary_nodes=SIZE_MAX;wrong.contributors.rigid_bodies=SIZE_MAX;
  wrong.auxiliary_rigid_primary_count=SIZE_MAX;wrong.auxiliary_rigid_primary_ids=reinterpret_cast<const std::uint64_t*>(1);
  EXPECT_EQ(s::Preflight(wrong).status,s::Status::ResourceLimit);
  EXPECT_EQ(s::BuildRigidOnly(wrong,{},memory.out,memory.scratch,&memory.result).status,s::Status::ResourceLimit);
}
TEST(Type25SearchContext, CompletePendingDescriptorAndAuxiliaryAliasesAreProtected) {
  Fixture f;std::uint64_t primary=100;auto in=Rigid(f,&primary);Storage memory(in);
  ASSERT_EQ(s::BuildRigidOnly(in,{},memory.out,memory.scratch,&memory.result).status,s::Status::Ok);
  const auto saved=memory.Bytes();in.auxiliary_rigid_primary_ids=static_cast<const std::uint64_t*>(memory.out.data());
  EXPECT_EQ(s::BuildRigidOnly(in,{},memory.out,memory.scratch,&memory.result).status,s::Status::InvalidInput);
  EXPECT_EQ(memory.Bytes(),saved);
  in=f.Input();in.contributors.tied_interfaces=1;in.contributors.cin_links=4;
  EXPECT_EQ(s::BuildGeometricBeforeTied(in,{},memory.out,memory.scratch,reinterpret_cast<s::GeometricSnapshot*>(memory.scratch.data())).status,s::Status::InvalidInput);
  EXPECT_EQ(memory.Bytes(),saved);
}
}
