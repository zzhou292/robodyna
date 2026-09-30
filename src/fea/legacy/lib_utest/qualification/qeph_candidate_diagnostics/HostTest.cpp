#include "Fixture.h"
namespace qeph_diagnostics_test {
TEST(QephCandidateDiagnostics,AllMasksSignedSumsAndSkinDefaultsMatchFrozenBits) {
  for(unsigned epoch=0;epoch<3;++epoch)for(unsigned mask=0;mask<8;++mask)for(bool assembled:{false,true}) {
    Fixture f;f.Reset(epoch,mask);
    const auto serial=f.Serial(epoch,f.input.law,assembled),actual=f.Staged(epoch,f.input.law,assembled);
    ASSERT_EQ(actual.status,q::BatchStatus::Success);SameControl(actual,serial);
    EXPECT_EQ(actual.diagnostics.internal_work[0],0);
    const double regrouped=(0x1p54-0x1p54)+1;
    EXPECT_NE(Bits(actual.diagnostics.internal_work[0]),Bits(regrouped));
  }
  Fixture skins;skins.Reset(0);
  for(unsigned p=0;p<Parents;++p) {
    skins.input.law[p]=fe::ShellSectionLaw::RigidSkin;
    auto& next=skins.host->slab[1].element[p];next={};
    ASSERT_EQ(q::InitializeHistory(skins.host->model.element[p].reference,{1e-6,1},next.proposed_history),q::Status::kSuccess);
  }
  const auto actual=skins.Staged(0,skins.input.law);SameControl(actual,skins.Serial(0,skins.input.law));
  EXPECT_EQ(actual.diagnostics.minimum_area_ratio,1);EXPECT_EQ(actual.diagnostics.minimum_thickness_ratio,1);
  EXPECT_EQ(actual.diagnostics.minimum_native_dt,0);
}
TEST(QephCandidateDiagnostics,PhasePriorityPartialFieldsMissingCatalogAndRetry) {
  for(unsigned fault=0;fault<8;++fault) {
    SCOPED_TRACE(fault);Fixture f;f.Reset(1);Fault(f,fault);
    const auto serial=f.Serial(1,f.input.law),actual=f.Staged(1,f.input.law);
    ASSERT_NE(actual.status,q::BatchStatus::Success);SameControl(actual,serial);
    if(fault==0) {EXPECT_EQ(actual.status,q::BatchStatus::ElementFailure);EXPECT_EQ(actual.element,3u);}
    if(fault==2)EXPECT_GT(actual.diagnostics.maximum_displacement,0);
  }
  Fixture f;f.Reset(1);f.host->candidate_status[2]=q::Status::kInvalidInput;
  SameControl(f.Staged(1,nullptr),f.Serial(1,nullptr));
  f.Reset(1);SameControl(f.Staged(1,nullptr),f.Serial(1,nullptr));
  f.Reset(1);const auto serial=f.Serial(1,f.input.law),actual=f.Staged(1,f.input.law);
  ASSERT_EQ(actual.status,q::BatchStatus::Success);SameControl(actual,serial);
}
TEST(QephCandidateDiagnostics,AccountedObserverTailScratchReuseAndStandaloneArithmetic) {
  b::Layout full;
  ASSERT_TRUE(full.InitializeMapped(324094,372435,std::size_t{2}<<30));
  // Later fixed observer reductions own a typed tail; this old serial/parallel
  // validation comparison still covers its unchanged arithmetic and scratch.
  EXPECT_EQ(full.bytes,1183491056u+32768u+sizeof(void*));
  b::Layout exact,short_cap;
  EXPECT_TRUE(exact.InitializeMapped(324094,372435,full.bytes));
  EXPECT_FALSE(short_cap.InitializeMapped(324094,372435,full.bytes-1));
  Fixture f;f.Reset(1);
  const auto offsets=std::array<std::uint32_t,Nodes+1>{f.host->assembly.offsets[0],f.host->assembly.offsets[1],
      f.host->assembly.offsets[2],f.host->assembly.offsets[3],f.host->assembly.offsets[4],f.host->assembly.offsets[5],f.host->assembly.offsets[6]};
  const auto before=f.Serial(1,f.input.law);
  for(unsigned n=0;n<Nodes;++n)f.host->assembly.node[n].value[0]=INFINITY;
  for(unsigned p=0;p<Parents;++p)f.host->assembly.parent[p].status=q::BatchStatus::ElementFailure;
  SameControl(f.Staged(1,f.input.law),before);
  for(unsigned n=0;n<=Nodes;++n)EXPECT_EQ(f.host->assembly.offsets[n],offsets[n]);
  // Legacy Measure's node and four kinetic folds still have their exact order.
  f.host->model.joined=false;
  for(unsigned n=0;n<Nodes;++n) {
    f.host->model.mass[n]=n+1;f.host->model.inertia[n]=n+.5;
    f.host->model.physical[n]=.25*n;f.host->model.added[n]=.5*n;
  }
  auto identity=Identity(1);identity.kinetic_available=true;
  b::Control old,next;old.diagnostics=next.diagnostics=identity;
  const auto view=f.fields.View(f.input,1);
  ASSERT_TRUE(frozen::Measure(f.host->model,f.host->slab[0],f.host->slab[1],view,old,f.input.law));
  ASSERT_TRUE(b::Measure(f.host->model,f.host->slab[0],f.host->slab[1],view,next,f.input.law));
  SameControl(next,old);
}
} // namespace qeph_diagnostics_test
