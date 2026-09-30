#include "WallRecurrenceTestFixture.h"
#include <limits>
#include <set>

namespace tl::qualification::qeph::wall_recurrence {
namespace r=recurrence;
TEST(QephWallRecurrenceModel, FrozenPenaltyFiniteWallAndIndependentNativeMassArea) {
  for(unsigned cells:{1u,2u}) {
    WallRecurrenceModel model; std::string error;
    ASSERT_TRUE(BuildWallRecurrenceModel(cells,model,error))<<error;
    ASSERT_TRUE(model.prepared()); const auto& m=model.native();
    EXPECT_EQ(m.nodes,2*(cells+1)); EXPECT_EQ(m.dictionary.size(),12*m.nodes+61*cells);
    EXPECT_EQ(model.law().maximum_penetration,MaximumDepth);
    EXPECT_EQ(model.law().parent_force_error,5e-7);
    EXPECT_EQ(model.law().parent_energy_error,1.2500000000000005e-12);
    EXPECT_EQ(model.law().stiffness_per_area,model.penalty_chain().back().upper);
    EXPECT_NE(model.law().stiffness_per_area,4e5);
    long double penalty=r::Density;
    const long double factors[]{r::Thickness,ImpactSpeed,ImpactSpeed,1.L/TargetDepth,1.L/TargetDepth};
    for(unsigned i=0;i<model.penalty_chain().size();++i) {
      if(i) penalty*=factors[i-1];
      EXPECT_LE(static_cast<long double>(model.penalty_chain()[i].lower),penalty);
      EXPECT_GE(static_cast<long double>(model.penalty_chain()[i].upper),penalty);
    }
    EXPECT_GT(model.law().stiffness_per_area,5.9e9); EXPECT_LT(model.law().stiffness_per_area,6e9);
    ASSERT_EQ(model.wall().faces().size(),2u); EXPECT_EQ(model.wall().wall_x(),0);
    EXPECT_TRUE(model.coverage().covered); EXPECT_EQ(model.coverage().mode,contact::PlanarWallBoxMode::Exact);
    EXPECT_EQ(model.coverage().physical.minimum.y,-MotionExtent);
    EXPECT_EQ(model.coverage().physical.maximum.z,MotionExtent);
    std::array<long double,r::MaxNodes> mass{},inertia{},area{};
    for(unsigned e=0;e<cells;++e) {
      const auto& ref=m.reference[e]; const auto& input=ref.data().input;
      const long double a=test::Area(ref),element_mass=r::Density*static_cast<long double>(r::Thickness)*a;
      test::Near(static_cast<double>(a),static_cast<long double>(r::Length)*r::Length,static_cast<double>(a));
      EXPECT_EQ(input.density,r::Density); EXPECT_EQ(input.thickness,r::Thickness);
      EXPECT_EQ(input.young_modulus,r::Young); EXPECT_EQ(input.poisson_ratio,r::Poisson);
      test::Certificate(model.reference().parent(e).area,a);
      test::Certificate(model.weights().parent(e).share,a/4);
      for(unsigned i=0;i<4;++i) {
        const auto n=m.connectivity[e][i];
        EXPECT_EQ(input.node_ids[i],100+n); EXPECT_EQ(input.position[i].x,-InitialGap);
        EXPECT_EQ(input.position[i].y,m.position[n].y); EXPECT_EQ(input.position[i].z,m.position[n].z);
        EXPECT_EQ(m.position[n].x,0);
        mass[n]+=element_mass/4; area[n]+=a/4;
        inertia[n]+=element_mass*(static_cast<long double>(r::Thickness)*r::Thickness+a)/48;
      }
    }
    for(unsigned n=0;n<m.nodes;++n) {
      const double expected_y_one[]{-.01,-.01,.01,.01};
      const double expected_y_two[]{-.02,-.02,0,0,.02,.02};
      EXPECT_EQ(m.position[n].y,cells==1?expected_y_one[n]:expected_y_two[n]);
      EXPECT_EQ(m.position[n].z,n%2?.01:-.01);
      test::Near(m.mass[n],mass[n],m.mass[n]); test::Near(m.inertia[n],inertia[n],m.inertia[n]);
      test::Certificate(model.weights().node(n).area,area[n]);
      test::Certificate(model.touching().nodes[n].stiffness,area[n]*model.law().stiffness_per_area);
      test::Near(model.mass_rates()[n].value,model.touching().nodes[n].stiffness.value/static_cast<long double>(m.mass[n]));
      EXPECT_TRUE(model.touching().nodes[n].touching_or_penetrating);
      EXPECT_EQ(model.touching().nodes[n].force.value,0); EXPECT_EQ(model.touching().nodes[n].potential.value,0);
    }
    EXPECT_LE(model.rate_spread_upper(),MassRateSpreadLimit); EXPECT_GT(model.maximum_frequency(),0);
    if(cells==2) {
      EXPECT_EQ(m.mass[2],2*m.mass[0]); EXPECT_EQ(m.inertia[2],2*m.inertia[0]);
      EXPECT_EQ(model.touching().nodes[2].stiffness.value,2*model.touching().nodes[0].stiffness.value);
    }
  }
}
TEST(QephWallRecurrenceModel, CompleteContactKickUsesPhysicalNodeAndDictionaryScales) {
  for(unsigned cells:{1u,2u}) {
    WallRecurrenceModel model; std::string error; ASSERT_TRUE(BuildWallRecurrenceModel(cells,model,error))<<error;
    const auto& m=model.native(); const auto size=m.dictionary.size();
    for(double h:r::Steps) {
      Eigen::MatrixXd inactive,active;
      ASSERT_TRUE(BuildContactKick(model,h,ContactBranch::Inactive,inactive,error));
      EXPECT_EQ(inactive,Eigen::MatrixXd::Identity(size,size));
      ASSERT_TRUE(BuildContactKick(model,h,ContactBranch::Active,active,error));
      for(unsigned row=0;row<size;++row) for(unsigned column=0;column<size;++column) {
        long double expected=row==column?1:0;
        const auto& v=m.dictionary[row]; const auto& x=m.dictionary[column];
        if(v.group==r::Group::Velocity&&v.component==0&&x.group==r::Group::Position&&x.component==0&&v.entity==x.entity) {
          long double node_area=0;
          for(unsigned e=0;e<cells;++e) for(unsigned n:m.connectivity[e]) if(n==v.entity) node_area+=test::Area(m.reference[e])/4;
          expected=-static_cast<long double>(h)*model.law().stiffness_per_area*node_area/m.mass[v.entity]*x.scale/v.scale;
          EXPECT_GT(std::abs(expected),32*r::MatrixTolerance);
        }
        test::Near(active(row,column),expected);
      }
      Eigen::MatrixXd shell=Eigen::MatrixXd::Identity(size,size);
      const auto x=model.coordinate(r::Group::Position,0,0),v=model.coordinate(r::Group::Velocity,0,0);
      shell(x,v)=h*m.dictionary[v].scale/m.dictionary[x].scale;
      Eigen::MatrixXd combined; ASSERT_TRUE(BuildContactBranch(model,h,ContactBranch::Active,shell,combined,error));
      EXPECT_EQ(combined,shell*active);
      EXPECT_GT((combined-active*shell).cwiseAbs().maxCoeff(),1e-12);
    }
  }
}
TEST(QephWallRecurrenceModel, SignConeDirectionsAreDistinctStrictAndUseNativeHistoryCoordinates) {
  WallRecurrenceModel model; std::string error; ASSERT_TRUE(BuildWallRecurrenceModel(2,model,error))<<error;
  for(auto branch:{ContactBranch::Inactive,ContactBranch::Active}) {
    std::vector<ContactDirection> directions; ASSERT_TRUE(SignConeDirections(model,branch,directions,error));
    EXPECT_EQ(directions.size(),model.native().nodes+7);
    std::set<std::string> names;
    for(const auto& d:directions) {
      EXPECT_TRUE(names.insert(d.name).second); EXPECT_TRUE(d.value.allFinite()); EXPECT_LE(d.value.cwiseAbs().maxCoeff(),1);
      for(unsigned n=0;n<model.native().nodes;++n) {
        const double x=d.value[model.coordinate(r::Group::Position,n,0)];
        EXPECT_GE(std::abs(x),.5); EXPECT_EQ(x>0,branch==ContactBranch::Active);
      }
    }
    EXPECT_EQ(directions[directions.size()-3].value[model.coordinate(r::Group::History,0,5)],1);
    EXPECT_EQ(directions[directions.size()-2].value[model.coordinate(r::Group::History,0,13)],1);
    EXPECT_EQ(directions.back().value[model.coordinate(r::Group::ForceCache,0,0)],1);
  }
}
TEST(QephWallRecurrenceModel, InvalidModelStepBranchAndLateCompositionPreserveOutputs) {
  WallRecurrenceModel model; std::string error; ASSERT_TRUE(BuildWallRecurrenceModel(1,model,error))<<error;
  const auto before=test::Bytes(&model,sizeof(model));
  EXPECT_FALSE(BuildWallRecurrenceModel(3,model,error)); EXPECT_EQ(test::Bytes(&model,sizeof(model)),before);
  Eigen::MatrixXd out=Eigen::MatrixXd::Constant(3,3,19),held=out;
  EXPECT_FALSE(BuildContactKick(model,0,ContactBranch::Active,out,error)); EXPECT_EQ(out,held);
  EXPECT_FALSE(BuildContactKick(model,r::H0,static_cast<ContactBranch>(19),out,error)); EXPECT_EQ(out,held);
  EXPECT_FALSE(BuildContactKick(WallRecurrenceModel{},r::H0,ContactBranch::Active,out,error)); EXPECT_EQ(out,held);
  Eigen::MatrixXd huge=Eigen::MatrixXd::Constant(model.native().dictionary.size(),model.native().dictionary.size(),
                                               std::numeric_limits<double>::max());
  // A negative normal-velocity column makes contact feedback ADD to a DBL_MAX
  // position column, provoking late product overflow with finite operands.
  for(unsigned n=0;n<model.native().nodes;++n) huge.col(model.coordinate(r::Group::Velocity,n,0))*=-1;
  EXPECT_FALSE(BuildContactBranch(model,4*r::H0,ContactBranch::Active,huge,out,error)); EXPECT_EQ(out,held);
  ASSERT_TRUE(BuildContactKick(model,r::H0,ContactBranch::Active,out,error)); EXPECT_TRUE(error.empty());
}
} // namespace tl::qualification::qeph::wall_recurrence
