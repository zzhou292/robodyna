#include "lib_src/collision/NodalWallContactPoint.h"
#include "lib_src/collision/Q4ParametricContact.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <type_traits>

namespace {
namespace sc=tlfea::contact;
using Code=sc::NodalWallStatus;
using Result=sc::NodalWallResult;
static_assert(sizeof(sc::NodalWallWeights)<=20*1024,"Immutable prescribed weight storage must stay bounded");
static_assert(sizeof(Result)<=80*1024,"Prescribed output storage must stay bounded");
// Declared before execution. Source-tuple force/energy budgets remain the C5c
// absolute N/J budgets. The independent O(1) arithmetic oracle is 2e-12.
constexpr double ForceBudget=5e-7,EnergyBudget=1.2500000000000005e-12,Arithmetic=2e-12;
sc::NodalWallConfig Config() { return {0,16,.5,ForceBudget,EnergyBudget}; }
void Encloses(sc::Q4CertifiedIntegral c,long double truth) {
  EXPECT_LE(static_cast<long double>(c.lower),truth); EXPECT_GE(static_cast<long double>(c.upper),truth);
  EXPECT_GE(static_cast<long double>(c.error),std::abs(static_cast<long double>(c.value)-truth));
}
void Near(double value,long double truth) {
  EXPECT_LE(std::abs(static_cast<long double>(value)-truth),Arithmetic*(1+std::abs(truth)));
}
template<class T> auto Bytes(const T& object) {
  static_assert(std::is_trivially_copyable_v<T>);
  std::array<unsigned char,sizeof(T)> bytes; std::memcpy(bytes.data(),&object,sizeof(T)); return bytes;
}
template<class T> void Unchanged(const T& object,const std::array<unsigned char,sizeof(T)>& bytes) {
  EXPECT_EQ(std::memcmp(&object,bytes.data(),sizeof(T)),0);
}
void Same(sc::Q4CertifiedIntegral a,sc::Q4CertifiedIntegral b) {
  EXPECT_EQ(a.value,b.value); EXPECT_EQ(a.lower,b.lower); EXPECT_EQ(a.upper,b.upper); EXPECT_EQ(a.error,b.error);
}
void Unchanged(const sc::NodalWallWeights& actual,const sc::NodalWallWeights& saved) {
  // The active-capacity container owns shared immutable backing. Compare its
  // complete public contents, not the representation of ownership handles.
  ASSERT_EQ(actual.prepared(),saved.prepared());
  ASSERT_EQ(actual.global_node_count(),saved.global_node_count());
  ASSERT_EQ(actual.parent_count(),saved.parent_count()); ASSERT_EQ(actual.node_count(),saved.node_count());
  EXPECT_EQ(actual.owned_payload_bytes(),saved.owned_payload_bytes()); Same(actual.total_area(),saved.total_area());
  for(unsigned p=0;p<actual.parent_count();++p) EXPECT_EQ(Bytes(actual.parent(p)),Bytes(saved.parent(p)));
  for(unsigned n=0;n<actual.node_count();++n) EXPECT_EQ(Bytes(actual.node(n)),Bytes(saved.node(n)));
}
void Overlap(sc::Q4CertifiedIntegral a,sc::Q4CertifiedIntegral b) {
  EXPECT_LE(a.lower,b.upper); EXPECT_LE(b.lower,a.upper);
  EXPECT_LE(std::abs(static_cast<long double>(a.value)-b.value),static_cast<long double>(a.error)+b.error);
}
void Same(const Result& a,const Result& b) {
  ASSERT_EQ(a.node_count,b.node_count); ASSERT_EQ(a.parent_count,b.parent_count);
  EXPECT_EQ(a.valid,b.valid); EXPECT_EQ(a.base_epoch,b.base_epoch); EXPECT_EQ(a.attempt,b.attempt);
  Same(a.resultant,b.resultant); Same(a.potential,b.potential);
  for (unsigned p=0;p<a.parent_count;++p) {
    const auto& x=a.parents[p]; const auto& y=b.parents[p];
    EXPECT_EQ(x.parent_element_id,y.parent_element_id); EXPECT_EQ(x.parent_face_id,y.parent_face_id);
    EXPECT_EQ(x.feature_id,y.feature_id); EXPECT_EQ(x.family,y.family); EXPECT_EQ(x.arity,y.arity);
    Same(x.resultant,y.resultant); Same(x.potential,y.potential);
    for (unsigned n=0;n<4;++n) Same(x.force[n],y.force[n]);
  }
  for (unsigned n=0;n<a.node_count;++n) {
    const auto& x=a.nodes[n]; const auto& y=b.nodes[n];
    EXPECT_EQ(x.node,y.node); EXPECT_EQ(x.valid,y.valid); EXPECT_EQ(x.fixed,y.fixed);
    Same(x.force,y.force); Same(x.potential,y.potential); Same(x.stiffness,y.stiffness);
    EXPECT_EQ(x.force_world.x,y.force_world.x); EXPECT_EQ(x.force_world.y,y.force_world.y); EXPECT_EQ(x.force_world.z,y.force_world.z);
    EXPECT_EQ(x.row.stiffness[0],y.row.stiffness[0]); EXPECT_EQ(x.row.count,y.row.count);
  }
  EXPECT_EQ(a.wall_reaction.x,b.wall_reaction.x); EXPECT_EQ(a.wall_reaction.y,b.wall_reaction.y);
  EXPECT_EQ(a.wall_reaction.z,b.wall_reaction.z); EXPECT_EQ(a.wall_moment.x,b.wall_moment.x);
  EXPECT_EQ(a.wall_moment.y,b.wall_moment.y); EXPECT_EQ(a.wall_moment.z,b.wall_moment.z);
  EXPECT_EQ(a.surface_power,b.surface_power);
}

struct Mixed {
  std::array<double,18> reference{{-1,0,0, -1,0,1, -1,-2,0, -1,3,0, -1,2,1, -1,2,0}};
  std::array<double,18> position=reference,velocity{};
  std::array<double,6> inverse{{1,2,3,4,5,6}};
  std::array<std::uint8_t,6> fixed{};
  sc::SurfaceQ4 quad{{4,1,0,5},2000,200,0,0};
  sc::SurfaceTriangle triangle{{0,1,2},1000,100,0,0,sc::SurfaceInterpolation::kLinearTriangle};
  sc::Q4ParametricReference q4;
  sc::T3MaterialMeasure t3;
  sc::NodalWallWeights weights;
  sc::VectorView View(const std::array<double,18>& x) const { return {x.data(),6,3,1}; }
  sc::LumpedTranslationMassView Mass() const { return {inverse.data(),fixed.data(),6,19,sc::TranslationMassModel::kIsotropicLumped}; }
  bool Prepare() {
    if (q4.Initialize(View(reference),&quad,1).status!=sc::Q4ParametricStatus::Ok ||
        sc::PrepareT3MaterialMeasure(View(reference),triangle,&t3)!=sc::SurfaceMeasureStatus::Ok) return false;
    const sc::NodalWallParentInput parents[2]{{&q4,0,nullptr},{nullptr,0,&t3}};
    return weights.Initialize(6,parents,2).status==Code::Ok;
  }
  void Depth(double d) { for (unsigned n=0;n<6;++n) position[3*n]=d; }
  sc::NodalWallReport Evaluate(Result* output,sc::NodalWallConfig config=Config()) const {
    return sc::EvaluateNodalWallContact(weights,View(position),View(velocity),Mass(),config,23,output);
  }
};

// A strip of n native Q4 panels, not n^2 parents or a triangulated Q4 proxy.
// Reference panel area=1/n; refining the strip preserves total material area.
struct Strip {
  unsigned panels=1,nodes=4;
  std::array<double,3*sc::MaxNodalWallNodes> reference{},position{},velocity{};
  std::array<double,sc::MaxNodalWallNodes> inverse{};
  std::array<std::uint8_t,sc::MaxNodalWallNodes> fixed{};
  std::array<sc::SurfaceQ4,16> parent{};
  std::array<sc::Q4ParametricReference,16> q4{};
  std::array<sc::NodalWallParentInput,16> inputs{};
  sc::NodalWallWeights weights;
  sc::VectorView View(const std::array<double,3*sc::MaxNodalWallNodes>& x) const { return {x.data(),nodes,3,1}; }
  sc::LumpedTranslationMassView Mass() const { return {inverse.data(),fixed.data(),nodes,3,sc::TranslationMassModel::kIsotropicLumped}; }
  bool Prepare(unsigned count=1) {
    if (!count || count>16) return false;
    panels=count; nodes=2*(count+1); inverse.fill(1);
    for (unsigned side=0;side<2;++side) for (unsigned i=0;i<=count;++i) {
      const auto node=side*(count+1)+i;
      reference[3*node]=-1; reference[3*node+1]=static_cast<double>(i)/count; reference[3*node+2]=side;
    }
    position=reference;
    for (unsigned i=0;i<count;++i) {
      parent[i]={{count+2+i,count+1+i,i,i+1},1000+i,500+i,0,0};
      if (q4[i].Initialize(View(reference),&parent[i],1).status!=sc::Q4ParametricStatus::Ok) return false;
      inputs[i]={&q4[i],0,nullptr};
    }
    return weights.Initialize(nodes,inputs.data(),count).status==Code::Ok;
  }
  void Affine(double amplitude,double cut) {
    for (unsigned n=0;n<nodes;++n) position[3*n]=amplitude*(position[3*n+1]-cut);
  }
  sc::NodalWallReport Evaluate(Result* output) const {
    return sc::EvaluateNodalWallContact(weights,View(position),View(velocity),Mass(),{0,1,2,ForceBudget,EnergyBudget},7,output);
  }
};

TEST(NodalWallContact, ImmutableNativeParentSharesAndSortedSourceIdentity) {
  Mixed f; ASSERT_TRUE(f.Prepare());
  EXPECT_STREQ(sc::NodalWallContactModel,"reference-area-lumped-nodal-wall-v1");
  RecordProperty("weight_storage_bytes",static_cast<int>(sizeof(sc::NodalWallWeights)));
  RecordProperty("result_storage_bytes",static_cast<int>(sizeof(Result)));
  ASSERT_EQ(f.weights.parent_count(),2u); ASSERT_EQ(f.weights.node_count(),5u);
  EXPECT_EQ(f.weights.global_node_count(),6u); // Node3 is deliberately unused.
  EXPECT_EQ(f.weights.parent(0).parent_element_id,100u); EXPECT_EQ(f.weights.parent(0).arity,3u);
  EXPECT_EQ(f.weights.parent(1).parent_element_id,200u); EXPECT_EQ(f.weights.parent(1).arity,4u);
  Encloses(f.weights.parent(0).area,1); Encloses(f.weights.parent(0).share,1.L/3);
  Encloses(f.weights.parent(1).area,2); Encloses(f.weights.parent(1).share,.5);
  Encloses(f.weights.total_area(),3);
  const unsigned ids[]{0,1,2,4,5}; const long double areas[]{5.L/6,5.L/6,1.L/3,.5,.5};
  for (unsigned n=0;n<5;++n) { EXPECT_EQ(f.weights.node(n).node,ids[n]); Encloses(f.weights.node(n).area,areas[n]); }
  const auto saved=f.weights;
  const sc::NodalWallParentInput reordered[2]{{nullptr,0,&f.t3},{&f.q4,0,nullptr}};
  sc::NodalWallWeights other; ASSERT_EQ(other.Initialize(6,reordered,2).status,Code::Ok);
  for (unsigned n=0;n<5;++n) Same(other.node(n).area,f.weights.node(n).area);
  f.reference[0]+=20; // Successful preparation owns copied areas/IDs, no pointer to these inputs.
  Unchanged(f.weights,saved);
  const sc::NodalWallParentInput duplicate[2]{{&f.q4,0,nullptr},{&f.q4,0,nullptr}};
  EXPECT_EQ(f.weights.Initialize(6,duplicate,2).status,Code::DuplicateParent); Unchanged(f.weights,saved);
  EXPECT_EQ(f.weights.Initialize(6,duplicate,129).status,Code::Capacity); Unchanged(f.weights,saved);
  EXPECT_EQ(f.weights.Initialize(129,duplicate,1).status,Code::Capacity); Unchanged(f.weights,saved);
  EXPECT_NE(f.weights.Initialize(6,nullptr,0).status,Code::Ok); Unchanged(f.weights,saved);
}

TEST(NodalWallContact, SharedNodesHaveOneForceAndSeparateParentEnergyBudgets) {
  Mixed f; ASSERT_TRUE(f.Prepare()); f.Depth(.125);
  for (unsigned n=0;n<6;++n) f.velocity[3*n]=static_cast<double>(n)-1;
  Result result; ASSERT_EQ(f.Evaluate(&result).status,Code::Ok);
  ASSERT_TRUE(result.valid); ASSERT_EQ(result.node_count,5u);
  Encloses(result.parents[0].resultant,2); Encloses(result.parents[0].potential,.125);
  Encloses(result.parents[1].resultant,4); Encloses(result.parents[1].potential,.25);
  Encloses(result.resultant,6); Encloses(result.potential,.375);
  EXPECT_EQ(result.parents[0].force[3].value,0); // Native T3 has no fourth force.
  long double expected_power=0;
  const long double areas[]{5.L/6,5.L/6,1.L/3,.5,.5};
  for (unsigned i=0;i<result.node_count;++i) {
    const auto& node=result.nodes[i]; const auto n=f.weights.node(i).node;
    EXPECT_EQ(node.node,n); Encloses(node.force,2*areas[i]); Encloses(node.potential,areas[i]/8);
    EXPECT_EQ(node.force_world.x,-node.force.value); EXPECT_EQ(node.force_world.y,0); EXPECT_EQ(node.force_world.z,0);
    ASSERT_TRUE(node.row.valid); ASSERT_EQ(node.row.count,1u); EXPECT_EQ(node.row.nodes[0],n);
    const long double rate=16*areas[i]*f.inverse[n];
    EXPECT_GE(static_cast<long double>(node.row.stiffness[0]),rate); Near(node.row.stiffness[0],rate);
    EXPECT_EQ(node.local_velocity_first_timestep,0); // Not min(isolated-share dt).
    expected_power-=2*areas[i]*f.velocity[3*n];
    sc::NodalWallPointResult assembled;
    ASSERT_EQ(sc::EvaluateNodalWallPoint(f.weights.node(i),f.View(f.position).at(n),f.View(f.velocity).at(n),
        f.Mass(),Config(),23,&assembled).status,Code::Ok);
    Overlap(assembled.force,node.force); Overlap(assembled.potential,node.potential);
  }
  Near(result.wall_reaction.x,6); Near(result.wall_moment.y,8.L/3); Near(result.wall_moment.z,-8.L/3);
  Near(result.surface_power,expected_power);
  const sc::NodalWallParentInput reversed[2]{{nullptr,0,&f.t3},{&f.q4,0,nullptr}};
  ASSERT_EQ(f.weights.Initialize(6,reversed,2).status,Code::Ok);
  Result reordered; ASSERT_EQ(f.Evaluate(&reordered).status,Code::Ok); Same(result,reordered);
  // Cyclic native connectivity changes no physical node or area contribution.
  // Each geometry uses a fresh immutable Q4 reference; prepared references do
  // not support replacement, unlike the staged nodal weight container.
  Mixed rotated; rotated.position=f.position; rotated.velocity=f.velocity;
  std::rotate(rotated.quad.nodes,rotated.quad.nodes+1,rotated.quad.nodes+4);
  std::rotate(rotated.triangle.nodes,rotated.triangle.nodes+1,rotated.triangle.nodes+3);
  ASSERT_TRUE(rotated.Prepare()); Result cyclic; ASSERT_EQ(rotated.Evaluate(&cyclic).status,Code::Ok);
  Overlap(result.resultant,cyclic.resultant); Overlap(result.potential,cyclic.potential);
  for (unsigned n=0;n<result.node_count;++n) {
    EXPECT_EQ(result.nodes[n].node,cyclic.nodes[n].node);
    Overlap(result.nodes[n].force,cyclic.nodes[n].force);
  }
  double parent_error_upper=0;
  for (unsigned p=0;p<result.parent_count;++p) {
    EXPECT_LE(result.parents[p].resultant.error,ForceBudget); EXPECT_LE(result.parents[p].potential.error,EnergyBudget);
    ASSERT_TRUE(sc::q4_bounds::AddScalar(parent_error_upper,result.parents[p].potential.error,true,&parent_error_upper));
  }
  EXPECT_LE(result.potential.error,parent_error_upper+128*std::numeric_limits<double>::epsilon());
}

TEST(NodalWallContact, SourceScaleTupleKeepsDeclaredParentBudgetsAndPhysicalMoments) {
  Strip strip; ASSERT_TRUE(strip.Prepare());
  constexpr double width=.04,height=.05,depth=.00025,kappa=4e5;
  // Dimensional analytic fixture, not authentication or mechanics admission of
  // the source part. Its nonbinary area has the actual represented dimensions.
  for (unsigned n=0;n<strip.nodes;++n) {
    strip.reference[3*n+1]*=width; strip.reference[3*n+2]*=height;
    strip.position[3*n]=depth;
    strip.position[3*n+1]=strip.reference[3*n+1]+.3;
    strip.position[3*n+2]=strip.reference[3*n+2]-.2;
    strip.inverse[n]=250;
  }
  sc::Q4ParametricReference scaled_reference;
  ASSERT_EQ(scaled_reference.Initialize(strip.View(strip.reference),&strip.parent[0],1).status,sc::Q4ParametricStatus::Ok);
  const sc::NodalWallParentInput scaled_input{&scaled_reference,0,nullptr};
  ASSERT_EQ(strip.weights.Initialize(strip.nodes,&scaled_input,1).status,Code::Ok);
  Result result;
  ASSERT_EQ(sc::EvaluateNodalWallContact(strip.weights,strip.View(strip.position),strip.View(strip.velocity),
      strip.Mass(),{0,kappa,.0005,ForceBudget,EnergyBudget},7,&result).status,Code::Ok);
  const long double area=static_cast<long double>(width)*height;
  const long double force=static_cast<long double>(kappa)*area*depth;
  const long double energy=.5L*force*depth;
  Encloses(result.resultant,force); Encloses(result.potential,energy);
  EXPECT_LE(result.parents[0].resultant.error,ForceBudget);
  EXPECT_LE(result.parents[0].potential.error,EnergyBudget);
  long double moment_y=0,moment_z=0;
  for (unsigned n=0;n<strip.nodes;++n) {
    Encloses(result.nodes[n].force,force/4);
    EXPECT_LE(result.parents[0].force[n].error,ForceBudget);
    moment_y+=strip.position[3*n+2]*force/4;
    moment_z-=strip.position[3*n+1]*force/4;
    const long double rate=static_cast<long double>(kappa)*area*250/4;
    EXPECT_GE(static_cast<long double>(result.nodes[n].row.stiffness[0]),rate);
    Near(result.nodes[n].row.stiffness[0],rate);
  }
  Near(result.wall_moment.y,moment_y); Near(result.wall_moment.z,moment_z);
  // Separate warped and exactly edge-on current configurations. A raw nodal
  // spring retains its reference weight; actual finite-wall coverage and swept
  // motion remain the separate future adapter's gate.
  for (bool edge_on:{false,true}) {
    for (unsigned n=0;n<strip.nodes;++n) {
      strip.position[3*n]=depth*(1+static_cast<double>(n*n)/16);
      strip.position[3*n+1]=.3+(edge_on?0:static_cast<double>(n%2)*.01);
      strip.position[3*n+2]=-.2+static_cast<double>(n/2)*.02;
    }
    ASSERT_EQ(sc::EvaluateNodalWallContact(strip.weights,strip.View(strip.position),strip.View(strip.velocity),
        strip.Mass(),{0,kappa,.0005,ForceBudget,EnergyBudget},7,&result).status,Code::Ok);
    moment_y=moment_z=0;
    for (unsigned n=0;n<strip.nodes;++n) {
      const long double nodal_force=static_cast<long double>(kappa)*area/4*strip.position[3*n];
      Encloses(result.nodes[n].force,nodal_force);
      moment_y+=strip.position[3*n+2]*nodal_force;
      moment_z-=strip.position[3*n+1]*nodal_force;
    }
    Near(result.wall_moment.y,moment_y); Near(result.wall_moment.z,moment_z);
  }
}

TEST(NodalWallContact, PointUsesGenuineMassAndExplicitFixedTouchConvention) {
  Mixed f; ASSERT_TRUE(f.Prepare()); auto config=Config();
  const sc::NodalWallNodeWeight weight{4,{.5,.5,.5,0}};
  sc::NodalWallPointResult output;
  for (double depth:{-.125,0.,.125}) {
    ASSERT_EQ(sc::EvaluateNodalWallPoint(weight,{depth,.25,.5},{.5,3,-2},f.Mass(),config,8,&output).status,Code::Ok);
    const long double positive=depth>0?depth:0;
    Encloses(output.force,8*positive); Encloses(output.potential,4*positive*positive);
    EXPECT_EQ(output.touching_or_penetrating,depth>=0);
    EXPECT_EQ(output.row.base_epoch,19u); EXPECT_EQ(output.row.attempt,8u);
    Near(output.row.stiffness[0],40); Near(output.surface_power,-4*positive);
  }
  const auto original=output; f.inverse[4]=.25;
  ASSERT_EQ(sc::EvaluateNodalWallPoint(weight,{.125,.25,.5},{.5,3,-2},f.Mass(),config,8,&output).status,Code::Ok);
  Same(output.force,original.force); Same(output.potential,original.potential);
  Near(output.row.stiffness[0],2); Near(output.local_velocity_first_timestep,1.6L/std::sqrt(2.L));
  f.fixed[4]=1; f.inverse[4]=0;
  ASSERT_EQ(sc::EvaluateNodalWallPoint(weight,{0,0,0},{},f.Mass(),config,8,&output).status,Code::Ok);
  EXPECT_TRUE(output.fixed); EXPECT_TRUE(output.touching_or_penetrating); EXPECT_EQ(output.force.value,0);
  EXPECT_FALSE(output.row.valid); EXPECT_EQ(output.local_velocity_first_timestep,0);
  const auto saved=Bytes(output);
  EXPECT_EQ(sc::EvaluateNodalWallPoint(weight,{.125,0,0},{},f.Mass(),config,8,&output).status,Code::FixedPenetration);
  Unchanged(output,saved);
  EXPECT_EQ(sc::EvaluateNodalWallPoint(weight,{-1,0,0},{1,0,0},f.Mass(),config,8,&output).status,Code::FixedMotion);
  Unchanged(output,saved);
}

TEST(NodalWallContact, CoarsePartialActivationIsAnExplicitDifferentModelForQ4AndT3) {
  Strip strip; ASSERT_TRUE(strip.Prepare()); strip.Affine(1,.5);
  Result quad; ASSERT_EQ(strip.Evaluate(&quad).status,Code::Ok);
  Encloses(quad.resultant,1.L/4); Encloses(quad.potential,1.L/16);
  EXPECT_GT(quad.resultant.lower,1.L/8); EXPECT_GT(quad.potential.lower,1.L/48);
  // Independent exact integral truths for unit-area g=u-1/2, not a relaxed
  // comparison to the old certificate. Nodal force and U differ by 2x/3x.
  Mixed f; ASSERT_TRUE(f.Prepare());
  const sc::NodalWallParentInput input{nullptr,0,&f.t3};
  sc::NodalWallWeights weights; ASSERT_EQ(weights.Initialize(6,&input,1).status,Code::Ok);
  f.Depth(-1); f.position[0]=1;
  Result triangle;
  ASSERT_EQ(sc::EvaluateNodalWallContact(weights,f.View(f.position),f.View(f.velocity),f.Mass(),
      {0,1,2,ForceBudget,EnergyBudget},9,&triangle).status,Code::Ok);
  Encloses(triangle.resultant,1.L/3); Encloses(triangle.potential,1.L/6);
  // Unit-area T3, one +1 and two -1 corners: exact clipped integral R=1/12,U=1/48.
  EXPECT_GT(triangle.resultant.lower,1.L/12); EXPECT_GT(triangle.potential.lower,1.L/48);
  // A tiny positive corner remains visible to B even when a finite interior
  // Gauss rule sees no positive sample. This is detection, not force accuracy.
  strip.Affine(1,.99); ASSERT_EQ(strip.Evaluate(&quad).status,Code::Ok);
  EXPECT_GT(quad.resultant.lower,0); EXPECT_LT((1+1/std::sqrt(3.))/2-.99,0);
}

TEST(NodalWallContact, SpatialStripRefinementMeetsIndependentShrinkingBounds) {
  for (unsigned n:{4u,8u,16u}) {
    SCOPED_TRACE(n);
    Strip strip; ASSERT_TRUE(strip.Prepare(n)); strip.Affine(1,1./3);
    Result result; ASSERT_EQ(strip.Evaluate(&result).status,Code::Ok);
    // Independent composite trapezoid sum uses actual represented x, without
    // any prepared area or production accumulator. Total area and kappa are1.
    long double discrete_force=0,discrete_energy=0,coordinate_error=0;
    for (unsigned i=0;i<=n;++i) {
      const long double actual=strip.position[3*i];
      const long double exact=static_cast<long double>(i)/n-1.L/3;
      coordinate_error=std::max(coordinate_error,std::abs(actual-exact));
      const long double positive=std::max(0.L,actual),weight=(i==0||i==n?.5L:1.L)/n;
      discrete_force+=weight*positive; discrete_energy+=weight*.5L*positive*positive;
    }
    Encloses(result.resultant,discrete_force); Encloses(result.potential,discrete_energy);
    const long double force_truth=2.L/9,energy_truth=4.L/81;
    const long double f_allow=result.resultant.error+coordinate_error;
    const long double u_allow=result.potential.error+(1+coordinate_error)*coordinate_error;
    EXPECT_GE(static_cast<long double>(result.resultant.value)-force_truth,-f_allow);
    EXPECT_LE(static_cast<long double>(result.resultant.value)-force_truth,1.L/(8*n*n)+f_allow);
    EXPECT_GE(static_cast<long double>(result.potential.value)-energy_truth,-u_allow);
    EXPECT_LE(static_cast<long double>(result.potential.value)-energy_truth,1.L/(12*n*n)+u_allow);
    RecordProperty("panels_"+std::to_string(n)+"_force",std::to_string(result.resultant.value));
    RecordProperty("panels_"+std::to_string(n)+"_energy",std::to_string(result.potential.value));
  }
}

TEST(NodalWallContact, VirtualWorkUsesBothEnergyCertificatesAndActualNodalDisplacement) {
  Mixed f; ASSERT_TRUE(f.Prepare()); f.Depth(.125);
  Result base; ASSERT_EQ(f.Evaluate(&base).status,Code::Ok);
  for (double h:{std::ldexp(1.,-12),std::ldexp(1.,-13)}) {
    Result plus,minus;
    f.position[0]=.125+h; ASSERT_EQ(f.Evaluate(&plus).status,Code::Ok);
    f.position[0]=.125-h; ASSERT_EQ(f.Evaluate(&minus).status,Code::Ok);
    f.position[0]=.125;
    sc::Q4IntegralInterval lower,upper,derivative;
    ASSERT_TRUE(sc::q4_bounds::Difference(plus.potential.lower,minus.potential.upper,&lower));
    ASSERT_TRUE(sc::q4_bounds::Difference(plus.potential.upper,minus.potential.lower,&upper));
    ASSERT_TRUE(sc::q4_bounds::Scale({lower.lower,upper.upper},1/(2*h),&derivative));
    EXPECT_LE(derivative.lower,5.L/3); EXPECT_GE(derivative.upper,5.L/3);
    EXPECT_LE(derivative.upper-derivative.lower,1e-8); // Predeclared finite propagated-error budget, N.
    Near((plus.potential.value-minus.potential.value)/(2*h),5.L/3);
    Near(-base.nodes[0].force_world.x,5.L/3);
    // Same active set, exact dyadic displacement and quadratic potential: zero
    // centered truncation error. This is a work check, not temporal integration.
  }
  f.position[1]+=1; f.position[2]-=2;
  Result moved; ASSERT_EQ(f.Evaluate(&moved).status,Code::Ok);
  Same(moved.potential,base.potential); Same(moved.resultant,base.resultant);
}

TEST(NodalWallContact, LateParentBudgetMassAndCapFailuresPreserveWholePublishedResult) {
  Mixed f; ASSERT_TRUE(f.Prepare()); f.Depth(.125);
  Result output; ASSERT_EQ(f.Evaluate(&output).status,Code::Ok); const auto saved=Bytes(output);
  f.Depth(-.125); f.position[12]=.125; // First sorted T3 wholly inactive; last Q4 has positive pressure.
  auto config=Config(); config.parent_energy_error=1e-30;
  auto report=f.Evaluate(&output,config);
  EXPECT_EQ(report.status,Code::Accuracy); EXPECT_EQ(report.parent,1u); Unchanged(output,saved);
  f.inverse[4]=0;
  report=f.Evaluate(&output); EXPECT_EQ(report.status,Code::MassFailure); EXPECT_EQ(report.parent,1u);
  EXPECT_EQ(report.node,4u); Unchanged(output,saved); f.inverse[4]=5;
  f.position[12]=.75;
  report=f.Evaluate(&output); EXPECT_EQ(report.status,Code::PenetrationLimit); EXPECT_EQ(report.parent,1u); Unchanged(output,saved);
  f.position[12]=.125;
  Result clean; ASSERT_EQ(f.Evaluate(&clean).status,Code::Ok);
  ASSERT_EQ(f.Evaluate(&output).status,Code::Ok); Same(output,clean);
  for (unsigned n=0;n<output.node_count;++n) EXPECT_EQ(output.nodes[n].row.base_epoch,19u);
}

TEST(NodalWallContact, MalformedCertificatesAndLateFinitePowerOverflowRejectAndRetry) {
  Mixed f; ASSERT_TRUE(f.Prepare());
  auto weight=f.weights.node(0); sc::NodalWallPointResult output;
  ASSERT_EQ(sc::EvaluateNodalWallPoint(weight,{.125,0,0},{},f.Mass(),Config(),2,&output).status,Code::Ok);
  const auto saved=Bytes(output);
  weight.area.error=0; // Nonzero exact-area radius cannot be understated.
  EXPECT_EQ(sc::EvaluateNodalWallPoint(weight,{.125,0,0},{},f.Mass(),Config(),2,&output).status,Code::InvalidReference);
  Unchanged(output,saved);
  weight={0,{1,1,1,0}}; auto config=Config(); config.stiffness_per_area=1e308; config.maximum_penetration=2;
  f.inverse[0]=1e-308;
  const auto report=sc::EvaluateNodalWallPoint(weight,{1,0,0},{2,0,0},f.Mass(),config,2,&output);
  EXPECT_EQ(report.status,Code::NonFiniteArithmetic); Unchanged(output,saved);
  // F=1e308 and U=5e307 are finite; real F.v overflows late. Clean reuse retains
  // the genuine large mass and succeeds without unsafe pointer/kernel faults.
  ASSERT_EQ(sc::EvaluateNodalWallPoint(weight,{1,0,0},{},f.Mass(),config,2,&output).status,Code::Ok);
  Encloses(output.force,static_cast<long double>(config.stiffness_per_area));
  Encloses(output.potential,.5L*config.stiffness_per_area);
  const auto clean=Bytes(output); f.fixed[0]=2;
  EXPECT_EQ(sc::EvaluateNodalWallPoint(weight,{1,0,0},{},f.Mass(),config,2,&output).status,Code::MassFailure);
  Unchanged(output,clean);
}
} // namespace
