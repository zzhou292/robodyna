#include "WallResponseTestFixture.h"
#include "WallResponseInternal.h"
#include <gtest/gtest.h>
#include <cstring>
#include <limits>
#include <vector>

namespace tl::qualification::qeph::wall_response {
namespace {
template<class T> std::vector<unsigned char> Bytes(const T& value) {
  const auto* first=reinterpret_cast<const unsigned char*>(&value);return {first,first+sizeof(value)};
}
TEST(WallResponseObserverCheck,ActualNativeModelScalesAndBoundedConfiguration) {
  for(unsigned cells:{1u,2u}) {
    Model model;std::string error;ASSERT_TRUE(BuildModel(cells,model,error))<<error;
    const long double area=cells*static_cast<long double>(free_response::Side)*free_response::Side;
    const long double mass=free_response::Density*free_response::Thickness*area;
    EXPECT_NEAR(model.energy(),static_cast<double>(.5L*mass*64),detail::Roundoff*model.energy());
    EXPECT_NEAR(model.momentum(),static_cast<double>(mass*8),detail::Roundoff*model.momentum());
    EXPECT_GT(model.force_scale(),0);EXPECT_LT(model.dictionary().size(),MaxFields);
    for(unsigned refinement:{1u,2u,4u}) {
      auto config=test::ConfigFor(cells,refinement);EXPECT_TRUE(ValidConfig(config));
      EXPECT_EQ(Steps(config),4096*refinement);EXPECT_EQ(SampleStride(config),16*refinement);
      config.selected_h=H0/2;EXPECT_EQ(Steps(config),8192*refinement);EXPECT_LE(Steps(config),MaxSteps);
    }
    auto invalid=test::ConfigFor(cells);invalid.refinement=3;EXPECT_FALSE(ValidConfig(invalid));EXPECT_EQ(Step(invalid),0);
    const auto before=model.energy();EXPECT_FALSE(BuildModel(3,model,error));EXPECT_EQ(model.energy(),before);EXPECT_TRUE(model.prepared());
  }
}
TEST(WallResponseObserverCheck,InitialVelocityIsPhysicalAndInitialHistoryIsExplicit) {
  Model model;std::string error;ASSERT_TRUE(BuildModel(1,model,error));const auto config=test::ConfigFor(1);
  auto input=test::EndpointAt(model,config,0);Sample sample;ASSERT_TRUE(ObserveEndpoint(model,config,input,sample,error))<<error;
  EXPECT_EQ(sample.time,0);EXPECT_EQ(sample.carried_velocity_time,0);EXPECT_EQ(sample.kick_dt,0);
  EXPECT_EQ(sample.synchronous_velocity,input.state.v);EXPECT_EQ(sample.wall_impulse,0);EXPECT_EQ(sample.synchronous_wall_impulse,0);
  EXPECT_LE(std::abs(sample.synchronous_kinetic[0]-model.energy()),sample.kinetic_error[0]+detail::Roundoff*model.energy());
  EXPECT_LE(sample.absolute_residual.upper,detail::Roundoff*model.energy());
  Summary summary;ASSERT_TRUE(StageSummary(model,config,{},sample,summary,error))<<error;EXPECT_TRUE(summary.initialized);
  const auto saved=Bytes(sample);input.state.v[0]+=1;EXPECT_FALSE(ObserveEndpoint(model,config,input,sample,error));EXPECT_EQ(Bytes(sample),saved);
  input=test::EndpointAt(model,config,0);port::HistoryValues values=input.elements[0].proposed_history.data();values.stress[4]=1;
  ASSERT_EQ(port::PreparePrescribedHistory(model.fields().reference[0],values,{0,0},input.elements[0].proposed_history),port::Status::kSuccess);
  EXPECT_FALSE(ObserveEndpoint(model,config,input,sample,error));EXPECT_EQ(Bytes(sample),saved);
}
TEST(WallResponseObserverCheck,TotalSignedRhsAndSeparateNativeContactKineticUncertainty) {
  Model model;std::string error;ASSERT_TRUE(BuildModel(2,model,error));const auto config=test::ConfigFor(2);
  auto input=test::EndpointAt(model,config,600);
  const auto n=model.fields().connectivity[0][1];ASSERT_EQ(n,model.fields().connectivity[1][0]);
  input.elements[0].internal_force[1].x=.001;input.elements[1].internal_force[0].x=-.00025;
  input.elements[1].internal_couple[0].y=3e-6;
  Sample sample;ASSERT_TRUE(ObserveEndpoint(model,config,input,sample,error))<<error;
  const long double rhs=-static_cast<long double>(input.contact.nodes[n].force.value)-.001L+.00025L;
  const long double expected=input.state.v[3*n]+.5L*Step(config)*rhs/model.fields().mass[n];
  EXPECT_LE(std::abs(static_cast<long double>(sample.synchronous_velocity[3*n])-expected),sample.velocity_error[3*n]);
  EXPECT_GT(sample.endpoint_native_force_error[3*n],0);EXPECT_GT(sample.endpoint_contact_force_error[3*n],0);
  EXPECT_GT(sample.endpoint_native_couple_error[3*n+1],0);EXPECT_LT(sample.synchronous_omega[3*n+1],0);
  EXPECT_GT(sample.kinetic_error[0],0);EXPECT_GT(sample.kinetic_error[1],0);
  const long double omitted=input.state.v[3*n]+.5L*Step(config)*(-.001L+.00025L)/model.fields().mass[n];
  EXPECT_GT(std::abs(omitted-sample.synchronous_velocity[3*n]),32*sample.velocity_error[3*n]);
  EXPECT_GT(sample.synchronous_wall_impulse,sample.wall_impulse);
  EXPECT_LE(std::abs((sample.synchronous_wall_impulse-sample.wall_impulse)-.5L*Step(config)*sample.contact.resultant.value),
            sample.synchronous_wall_impulse_error);
  ASSERT_TRUE(ValidateSample(model,config,sample,error))<<error;
  sample.endpoint_native_force_error[3*n]=0;EXPECT_FALSE(ValidateSample(model,config,sample,error));
}
TEST(WallResponseObserverCheck,LatePhaseRowAndOverflowFailuresPreserveWholeSample) {
  Model model;std::string error;ASSERT_TRUE(BuildModel(2,model,error));const auto config=test::ConfigFor(2);
  const auto input=test::EndpointAt(model,config,600);Sample output;ASSERT_TRUE(ObserveEndpoint(model,config,input,output,error));
  const auto saved=Bytes(output);
  for(unsigned fault=0;fault<5;++fault) {
    auto bad=input;
    if(fault==0)bad.carried_velocity_time=bad.time;
    if(fault==1)++bad.contact.nodes.back().row.base_epoch;
    if(fault==2)bad.elements[1].internal_force[3].z=std::numeric_limits<double>::infinity();
    if(fault==3)bad.state.v[3*(model.fields().nodes-1)]=1e200;
    if(fault==4)bad.contact.nodes.back().row.stiffness[1]=1;
    EXPECT_FALSE(ObserveEndpoint(model,config,bad,output,error))<<fault;EXPECT_EQ(Bytes(output),saved)<<fault;
  }
  EXPECT_TRUE(ObserveEndpoint(model,config,input,output,error));EXPECT_EQ(Bytes(output),saved);
}
TEST(WallResponseObserverCheck,RawTranslationAndQuaternionSignRemainObservable) {
  Model model;std::string error;ASSERT_TRUE(BuildModel(1,model,error));const auto config=test::ConfigFor(1);
  auto input=test::EndpointAt(model,config,Steps(config));Sample first;ASSERT_TRUE(ObserveEndpoint(model,config,input,first,error))<<error;
  EXPECT_GT(std::abs(first.state.x[0]-model.fields().initial_position[0])/free_response::Side,1e-3);
  EXPECT_LE(first.relative_displacement/free_response::Side,1e-3);EXPECT_LT(first.maximum_gap,0);EXPECT_LT(first.maximum_velocity,0);
  for(unsigned n=0;n<model.fields().nodes;++n)input.state.q[4*n]=-1;
  Sample opposite;ASSERT_TRUE(ObserveEndpoint(model,config,input,opposite,error))<<error;
  EXPECT_EQ(first.rotation_vector,opposite.rotation_vector);EXPECT_NE(first.state.q,opposite.state.q);
  const auto offset=model.node_fields(0).x;EXPECT_EQ(first.values[offset],first.state.x[0]);EXPECT_TRUE(model.dictionary()[offset].compare);
}
TEST(WallResponseObserverCheck,EveryEndpointEntryPeakReleaseAndRollbackAreIndependentOfDisplayCadence) {
  Model model;std::string error;ASSERT_TRUE(BuildModel(1,model,error));const auto config=test::ConfigFor(1);
  Summary summary,after_release;Sample sample;unsigned samples=0;
  for(unsigned epoch=0;epoch<=Steps(config);++epoch) {
    const auto input=test::EndpointAt(model,config,epoch);ASSERT_TRUE(ObserveEndpoint(model,config,input,sample,error))<<epoch<<' '<<error;
    Summary next;ASSERT_TRUE(StageSummary(model,config,summary,sample,next,error))<<epoch<<' '<<error;
    if(summary.phase!=Phase::Separated&&next.phase==Phase::Separated)after_release=next;
    summary=next;if(epoch%SampleStride(config)==0)++samples;
  }
  EXPECT_EQ(samples,SampleCount);ASSERT_TRUE(CompleteSummary(model,config,summary,error))<<error;
  EXPECT_EQ(summary.entry.after_epoch,197u);EXPECT_EQ(summary.entry.after_epoch,summary.entry.before_epoch+1);
  EXPECT_LT(summary.entry.after_epoch,summary.peak.after_epoch);EXPECT_LT(summary.peak.after_epoch,summary.exit.after_epoch);
  Interval depth,force;ASSERT_TRUE(PeakTruth(model,summary,depth,force,error));EXPECT_LE(depth.upper,.02);EXPECT_LE(force.upper,.02);
  auto wrong=summary;wrong.maximum_depth*=.5;EXPECT_FALSE(CompleteSummary(model,config,wrong,error));
  wrong=summary;wrong.maximum_resultant*=.5;EXPECT_FALSE(CompleteSummary(model,config,wrong,error));
  const auto saved=Bytes(summary);
  auto reentry=test::EndpointAt(model,config,after_release.last_epoch+1);
  for(unsigned n=0;n<model.fields().nodes;++n)reentry.state.x[3*n]=.0001;
  test::RefreshContact(model,reentry);Sample candidate;ASSERT_TRUE(ObserveEndpoint(model,config,reentry,candidate,error));
  EXPECT_FALSE(StageSummary(model,config,after_release,candidate,summary,error));EXPECT_EQ(error,"Reentry or nonoutgoing separated state");EXPECT_EQ(Bytes(summary),saved);
  reentry=test::EndpointAt(model,config,after_release.last_epoch+2);ASSERT_TRUE(ObserveEndpoint(model,config,reentry,candidate,error));
  EXPECT_FALSE(StageSummary(model,config,after_release,candidate,summary,error));EXPECT_EQ(Bytes(summary),saved);
}
} // namespace
} // namespace tl::qualification::qeph::wall_response
