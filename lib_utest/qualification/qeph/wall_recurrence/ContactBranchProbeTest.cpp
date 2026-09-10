#include "WallRecurrenceTestFixture.h"
#include "MovingNativeProbe.h"
#include <limits>

namespace tl::qualification::qeph::wall_recurrence {
namespace r=recurrence;
TEST(QephWallContactProbe, ActualHostCertificatesAndNativeKickUseEachSharedNodeOnce) {
  for(unsigned cells:{1u,2u}) {
    WallRecurrenceModel model; std::string error; ASSERT_TRUE(BuildWallRecurrenceModel(cells,model,error))<<error;
    const auto& m=model.native();
    for(double boost:VelocityBaselines) for(double sign:{-1.,1.}) {
      SCOPED_TRACE(cells);
      SCOPED_TRACE(boost);
      SCOPED_TRACE(sign);
      Eigen::VectorXd input=Eigen::VectorXd::Zero(m.dictionary.size());
      std::array<long double,r::MaxNodes> area{},depth{};
      for(unsigned e=0;e<cells;++e) for(unsigned n:m.connectivity[e]) area[n]+=test::Area(m.reference[e])/4;
      for(unsigned n=0;n<m.nodes;++n) {
        const auto x=model.coordinate(r::Group::Position,n,0);
        input[x]=sign*r::Amplitudes[0]*(n==2?1:.5);
        // The actual represented input is the oracle operand, not an ideal
        // decimal preload. Its physical conversion is the map's public scale.
        depth[n]=input[x]*m.dictionary[x].scale;
      }
      ContactMapSample sample; ASSERT_TRUE(EvaluateContactNativeMap(model,r::H0,boost,input,sample,error))<<error;
      ASSERT_EQ(sample.nodes.size(),m.nodes); ASSERT_EQ(sample.parents.size(),cells);
      long double resultant=0,potential=0,power=0;
      for(unsigned n=0;n<m.nodes;++n) {
        const auto& point=sample.nodes[n]; const long double gap=std::max(0.L,depth[n]);
        const long double force=model.law().stiffness_per_area*area[n]*gap,energy=.5L*force*gap;
        test::Certificate(point.force,force); test::Certificate(point.potential,energy);
        test::Near(point.force_world.x,-force); EXPECT_EQ(point.force_world.y,0); EXPECT_EQ(point.force_world.z,0);
        EXPECT_EQ(point.node,n); EXPECT_EQ(point.base_epoch,1u); EXPECT_EQ(point.attempt,1u);
        EXPECT_EQ(point.touching_or_penetrating,sign>0); EXPECT_FALSE(point.fixed); EXPECT_TRUE(point.valid);
        test::Near(point.wall_reaction.x,force);
        test::Near(point.wall_moment.y,m.position[n].z*force);
        test::Near(point.wall_moment.z,-m.position[n].y*force);
        test::Near(point.surface_power,-force*boost);
        const auto v=model.coordinate(r::Group::Velocity,n,0),x=model.coordinate(r::Group::Position,n,0);
        const long double next_velocity=boost-r::H0*force/m.mass[n];
        test::Near(sample.state[v],next_velocity/m.dictionary[v].scale);
        test::Near(sample.state[x],(depth[n]+r::H0*next_velocity)/m.dictionary[x].scale);
        for(unsigned a=0;a<3;++a) {
          EXPECT_EQ(sample.state[model.coordinate(r::Group::Spin,n,a)],0);
          EXPECT_EQ(sample.state[model.coordinate(r::Group::OrientationTangent,n,a)],0);
        }
        resultant+=force; potential+=energy; power-=force*boost;
      }
      test::Certificate(sample.resultant,resultant); test::Certificate(sample.potential,potential);
      test::Near(sample.surface_power,power);
      for(unsigned e=0;e<cells;++e) {
        const auto& p=sample.parents[e]; long double total=0,energy=0;
        for(unsigned i=0;i<4;++i) {
          const auto n=m.connectivity[e][i]; const long double gap=std::max(0.L,depth[n]);
          const long double f=model.law().stiffness_per_area*test::Area(m.reference[e])/4*gap;
          test::Certificate(p.force[i],f); total+=f; energy+=.5L*f*gap;
        }
        test::Certificate(p.resultant,total); test::Certificate(p.potential,energy);
        EXPECT_LE(p.resultant.error,ParentForceError); EXPECT_LE(p.potential.error,ParentEnergyError);
      }
    }
  }
}
TEST(QephWallContactProbe, TouchingBaselineIsActualMovingNativeResultWithoutContactBias) {
  for(unsigned cells:{1u,2u}) {
    WallRecurrenceModel model; std::string error; ASSERT_TRUE(BuildWallRecurrenceModel(cells,model,error))<<error;
    const auto zero=Eigen::VectorXd::Zero(model.native().dictionary.size());
    for(double boost:VelocityBaselines) {
      ContactMapSample sample; Eigen::VectorXd shell;
      ASSERT_TRUE(EvaluateContactNativeMap(model,r::H0,boost,zero,sample,error))<<error;
      ASSERT_TRUE(r::NativeMapWithUniformVelocity(model.native(),r::H0,{boost,0,0},zero,shell,error))<<error;
      EXPECT_EQ(sample.state,shell); EXPECT_EQ(sample.resultant.value,0); EXPECT_EQ(sample.potential.value,0);
      for(const auto& p:sample.nodes) {
        EXPECT_EQ(p.force.value,0); EXPECT_EQ(p.potential.value,0); EXPECT_TRUE(p.touching_or_penetrating);
        EXPECT_GT(p.stiffness.lower,0); // Touching has a declared active derivative, despite zero force.
      }
      if(boost!=0) EXPECT_NE(sample.state[model.coordinate(r::Group::Position,0,0)],0);
    }
  }
}
TEST(QephWallContactProbe, FullNativeBranchesMatchPhysicalSignConesAtThreeBoosts) {
  for(unsigned cells:{1u,2u}) {
    WallRecurrenceModel model; std::string error; ASSERT_TRUE(BuildWallRecurrenceModel(cells,model,error))<<error;
    for(double boost:VelocityBaselines) {
      SCOPED_TRACE(cells);
      SCOPED_TRACE(boost);
      const auto shell=DifferentiateMoving(model.native(),r::H0,r::Amplitudes.back(),{boost,0,0});
      ASSERT_TRUE(shell.baseline_complete); ASSERT_TRUE(shell.derivative.complete)<<shell.derivative.diagnostic;
      for(auto branch:{ContactBranch::Inactive,ContactBranch::Active}) {
        const auto probe=ProbeContactBranch(model,r::H0,boost,branch,shell.derivative.full);
        ASSERT_TRUE(probe.baseline_complete)<<probe.diagnostic;
        ASSERT_TRUE(probe.complete)<<probe.diagnostic;
        EXPECT_TRUE(probe.passed)<<probe.diagnostic;
        EXPECT_EQ(probe.baseline.state,shell.baseline);
        ASSERT_EQ(probe.directions.size(),model.native().nodes+7);
        for(const auto& d:probe.directions) {
          SCOPED_TRACE(d.direction.name);
          EXPECT_EQ(d.completed_samples,3u); EXPECT_TRUE(d.complete); EXPECT_TRUE(d.passed)<<d.diagnostic;
          for(unsigned level=0;level<2;++level) {
            EXPECT_TRUE(d.quotients[level].allFinite()); EXPECT_GT(d.budget[level],0);
            EXPECT_LE(d.residual_upper[level],d.budget[level]);
          }
        }
        if(branch==ContactBranch::Active) {
          const auto& d=probe.directions.front();
          const Eigen::VectorXd omitted_contact=shell.derivative.full*d.direction.value;
          EXPECT_GT((d.quotients[1]-omitted_contact).cwiseAbs().maxCoeff(),d.budget[1]);
        }
      }
    }
  }
}
TEST(QephWallContactProbe, ContactThenLateNativeFailuresPreserveAllOutputAndRetry) {
  WallRecurrenceModel model; std::string error; ASSERT_TRUE(BuildWallRecurrenceModel(2,model,error))<<error;
  Eigen::VectorXd input=Eigen::VectorXd::Zero(model.native().dictionary.size());
  for(unsigned n=0;n<model.native().nodes;++n) input[model.coordinate(r::Group::Position,n,0)]=r::Amplitudes[0];
  ContactMapSample output;
  ASSERT_TRUE(EvaluateContactNativeMap(model,r::H0,8,input,output,error))<<error;
  const test::SampleSnapshot snapshot(output);
  EXPECT_FALSE(EvaluateContactNativeMap(model,0,8,input,output,error)); snapshot.Check(output);
  EXPECT_FALSE(EvaluateContactNativeMap(model,r::H0,std::numeric_limits<double>::quiet_NaN(),input,output,error));
  snapshot.Check(output);
  auto bad=input; bad[model.coordinate(r::Group::Position,0,0)]*=-1;
  EXPECT_FALSE(EvaluateContactNativeMap(model,r::H0,8,bad,output,error)); snapshot.Check(output);
  bad=input; bad[model.coordinate(r::Group::Position,0,1)]=100;
  EXPECT_FALSE(EvaluateContactNativeMap(model,r::H0,8,bad,output,error)); snapshot.Check(output);
  bad=input;
  for(unsigned n=0;n<model.native().nodes;++n) bad[model.coordinate(r::Group::Position,n,0)]=2*MaximumDepth/r::Length;
  EXPECT_FALSE(EvaluateContactNativeMap(model,r::H0,8,bad,output,error)); snapshot.Check(output);
  bad=input; bad[model.coordinate(r::Group::History,1,33)]=-2;
  EXPECT_FALSE(EvaluateContactNativeMap(model,r::H0,8,bad,output,error));
  EXPECT_EQ(error,"Native prescribed history rejected"); snapshot.Check(output);
  ContactMapSample clean;
  ASSERT_TRUE(EvaluateContactNativeMap(model,r::H0,8,input,clean,error))<<error;
  ASSERT_TRUE(EvaluateContactNativeMap(model,r::H0,8,input,output,error))<<error;
  EXPECT_EQ(output.state,clean.state); EXPECT_EQ(output.potential.value,clean.potential.value);
  ASSERT_EQ(output.nodes.size(),clean.nodes.size());
  for(unsigned n=0;n<output.nodes.size();++n) {
    EXPECT_EQ(output.nodes[n].force.value,clean.nodes[n].force.value);
    EXPECT_EQ(output.nodes[n].force.lower,clean.nodes[n].force.lower);
    EXPECT_EQ(output.nodes[n].force.upper,clean.nodes[n].force.upper);
    EXPECT_EQ(output.nodes[n].force.error,clean.nodes[n].force.error);
  }
  EXPECT_TRUE(error.empty());
}
} // namespace tl::qualification::qeph::wall_recurrence
