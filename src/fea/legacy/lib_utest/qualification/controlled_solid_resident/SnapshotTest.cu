// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Snapshot.h"
#include <cstdlib>
#include <filesystem>
#include <fstream>
namespace controlled_resident_test {
TEST_F(ControlledResidentCuda, SixteenCarriedStepsAcrossAllWorkerTypesExportCompleteNumericFields) {
  std::ofstream output;
  if(const auto* path=std::getenv("TL_CONTROLLED_PACKET_SNAPSHOT")){
    ASSERT_FALSE(std::filesystem::exists(path));output.open(path,std::ios::out|std::ios::binary);ASSERT_TRUE(output.good());}
  unsigned case_id=0;
  for(const auto units:{s::control::UnitScale{1,1,1},s::control::UnitScale{.001,1000,1}})for(bool collapsed:{false,true}){
    SCOPED_TRACE(case_id);Rig rig(units,collapsed);ASSERT_TRUE(rig.Initialize());
    ASSERT_EQ(rig.fixture.model.control_selection()->packets().size(),19u);
    ASSERT_EQ(rig.fixture.model.solid24().size(),8u);ASSERT_EQ(rig.fixture.model.solid18_law90().size(),8u);
    Results result(rig.fixture.model);s::BatchDiagnostics diagnostics;ASSERT_TRUE(rig.Read(result,diagnostics));
    if(output.is_open())snapshot::Emit(output,std::to_string(case_id)+".0",result,diagnostics);
    for(unsigned step=1;step<=16;++step){
      fe::NodalTrialToken token;fe::NodalAssemblyView assembly;fe::NodalPreparedView prepared;
      ASSERT_TRUE(rig.Begin(token,assembly));rig.CheckAssembly(token,assembly,result);ASSERT_FALSE(HasFailure());
      ASSERT_TRUE(rig.Prepare(token,assembly,prepared));s::BatchDiagnostics next;
      ASSERT_TRUE(Good(rig.batch.EvaluateCandidate(rig.owner,token,prepared,&next)));
      ASSERT_TRUE(Good(Peer::Commit(rig.batch,rig.owner,token,prepared,next)));
      ASSERT_TRUE(rig.Read(result,diagnostics));EXPECT_EQ(diagnostics.epoch,step);
      if(output.is_open())snapshot::Emit(output,std::to_string(case_id)+'.'+std::to_string(step),result,diagnostics);
    }
    ++case_id;
  }
  if(output.is_open()){output.flush();ASSERT_TRUE(output.good());}
}
} // namespace controlled_resident_test
