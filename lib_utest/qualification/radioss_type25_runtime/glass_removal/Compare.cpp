// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Rig.h"
#include "../../radioss_type25_local_geometry/Assertions.h"
#include "../../shell_failure_force/NativeAgreement.h"
#include <cstring>
namespace glass_removal_test {
namespace {
template<class T,std::size_t N> void Bits(const std::array<T,N>& a,const std::array<T,N>& b) {
  EXPECT_EQ(std::memcmp(a.data(),b.data(),N*sizeof(T)),0);
}
void Section(const fe::ShellBatchLayeredSection& a,const fe::ShellBatchLayeredSection& b) {
  ASSERT_EQ(a.law(),b.law());
  if(a.elastic()){
    ASSERT_NE(b.elastic(),nullptr);
    std::vector<double> x,y;
    for(unsigned i=0;i<3;++i){failure_force_test::Append(x,a.elastic()->point[i].stress);
      failure_force_test::Append(y,b.elastic()->point[i].stress);}
    failure_force_test::Exact(x,y);
  } else {
    ASSERT_NE(a.plastic(),nullptr);
    ASSERT_NE(b.plastic(),nullptr);
    failure_force_test::Exact(resident_tab1_test::Values(*a.plastic()),resident_tab1_test::Values(*b.plastic()));
  }
}
void Publication(const fe::NativeContactPublicationSnapshot& a,const fe::NativeContactPublicationSnapshot& b) {
  EXPECT_TRUE(fe::trial_identity::SameStamp(a.stamp,b.stamp));
  EXPECT_TRUE(fe::trial_identity::SameStamp(a.force_base_stamp,b.force_base_stamp));
  EXPECT_EQ(a.available,b.available);
  EXPECT_EQ(a.force_phase_available,b.force_phase_available);
  EXPECT_EQ(a.generation,b.generation);
  const auto& x=a.selectors;const auto& y=b.selectors;
  EXPECT_EQ(x.history,y.history);
  EXPECT_EQ(x.reference,y.reference);
  EXPECT_EQ(x.reference_generation,y.reference_generation);
  EXPECT_EQ(x.has_reference,y.has_reference);
  EXPECT_EQ(x.activity,y.activity);
  EXPECT_EQ(x.activity_generation,y.activity_generation);
  EXPECT_EQ(x.reference_activity_generation,y.reference_activity_generation);
}
}
void Same(const State& a,const State& b) {
  EXPECT_TRUE(fe::trial_identity::SameStamp(a.stamp,b.stamp));
  Bits(a.x,b.x);Bits(a.v,b.v);Bits(a.omega,b.omega);Bits(a.q,b.q);
  Bits(a.reaction,b.reaction);Bits(a.couple,b.couple);Bits(a.mass,b.mass);Bits(a.inertia,b.inertia);
  for(unsigned i=0;i<2;++i){
    SCOPED_TRACE(i);
    failure_force_test::Exact(failure_force_test::ForceValues(a.force[i]),failure_force_test::ForceValues(b.force[i]));
    failure_force_test::Exact(failure_force_test::Diagnostics(a.force[i].diagnostics),failure_force_test::Diagnostics(b.force[i].diagnostics));
    Section(a.section[i],b.section[i]);
    failure_force_test::Exact(resident_tab1_test::Values(a.failure[i]),resident_tab1_test::Values(b.failure[i]));
    EXPECT_EQ(a.flags[i],b.flags[i]);
    ASSERT_EQ(a.contacts[i].size(),b.contacts[i].size());
    for(std::size_t row=0;row<a.contacts[i].size();++row)type25_geometry_test::Same(a.contacts[i][row],b.contacts[i][row]);
    Publication(a.publication[i],b.publication[i]);
  }
  failure_force_test::Exact(failure_force_test::ForceValues(a.triangle_force),failure_force_test::ForceValues(b.triangle_force));
  failure_force_test::Exact(failure_force_test::Diagnostics(a.triangle_force.diagnostics),failure_force_test::Diagnostics(b.triangle_force.diagnostics));
  Section(a.triangle_section,b.triangle_section);
  failure_force_test::Exact(resident_tab1_test::Values(a.triangle_failure),resident_tab1_test::Values(b.triangle_failure));
}
} // namespace glass_removal_test
