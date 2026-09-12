// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/elements/ShellPhysicalOutputRanges.h"
namespace beam18_model_test {
TEST(Beam18Ledger, AllRetainedBeamRangesRejectOutputAliasing) {
  const Fixture f; const auto d=f.structural.Domain(); const auto map=f.structural.Map(d); const auto beam=f.Contributions(d);
  fe::NodalCoefficientLedger ledger; ASSERT_TRUE(ledger.InitializeWithBeams({{{&map}},&beam}));
  const auto& stored=*ledger.beam18(); const auto& model=*stored.model();
  auto disjoint=[&](const void* p,std::size_t n) {return fe::shell_physical_owner::OutputDisjoint(ledger,p,n);};
  double ordinary=0; EXPECT_TRUE(disjoint(&ordinary,sizeof(ordinary)));
  EXPECT_FALSE(disjoint(&stored,sizeof(stored))); EXPECT_FALSE(disjoint(&model,sizeof(model)));
  EXPECT_FALSE(disjoint(&model.parents()[model.parents().size()-1],sizeof(b::Parent)));
  EXPECT_FALSE(disjoint(&model.materials()[0],sizeof(b::MaterialRecord)));
  EXPECT_FALSE(disjoint(&stored.records()[stored.records().size()-1],sizeof(fe::Beam18NodeContribution)));
  const auto& curve=model.materials()[0].value.curve;
  EXPECT_FALSE(disjoint(curve.plastic_strain+curve.count-1,sizeof(double)));
  EXPECT_FALSE(disjoint(curve.yield_stress_pa+curve.count-1,sizeof(double)));
  EXPECT_FALSE(disjoint(&model.domain()->nodes()[model.domain()->node_count()-1],sizeof(fe::NodalDomainNode)));
}
} // namespace beam18_model_test
