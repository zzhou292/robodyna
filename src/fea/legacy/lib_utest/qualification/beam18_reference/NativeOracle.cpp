// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
extern "C" void beam18_reference_native(const double*,const double*,const double*,
    const double*,const double*,const int*,double*,double*,double*,double*,int*);
namespace beam18_test {
NativeResult Native(const beam::Input& input) {
  NativeResult result;
  double x[9];
  for (unsigned i=0;i<3;++i) {
    x[3*i]=input.position[i].x; x[3*i+1]=input.position[i].y; x[3*i+2]=input.position[i].z;
  }
  // HM_READ_BEAM:158-183, retained in native/source-manifest.json.
  // The selected node-defined profile has IBEAM_VECTOR=0; system N2 is 2.
  const auto third=input.source_node_id[2];
  const int n3=(!third || third==input.source_node_id[0] || third==input.source_node_id[1]) ? 2 : 3;
  beam18_reference_native(x,&input.radius,&input.density,&input.young,&input.poisson,
      &n3,result.values.data(),result.node_stiffness.data(),result.node_rotation.data(),
      &result.part_mass,&result.status);
  if (result.status) return result;
  const bool working=input.units==beam::WorkingUnits::TonneMillimetreSecond;
  const double mass=working?1000:1, inertia=working?1000*.001*.001:1;
  result.values[27]=result.values[20]*mass;
  result.values[28]=result.values[21]*inertia;
  result.values[29]=result.values[22]*mass;
  result.values[30]=result.values[23]*inertia;
  result.values[31]=result.values[24]*mass;
  return result;
}
void Compare(const beam::Reference& actual,const NativeResult& expected) {
  ASSERT_EQ(expected.status,0);
  const auto values=Values(actual);
  for (unsigned i=0;i<values.size();++i) {
    SCOPED_TRACE(i);
    EXPECT_NEAR(values[i],expected.values[i],3e-13*std::max(std::abs(expected.values[i]),1e-30));
  }
  EXPECT_EQ(expected.node_stiffness[0],expected.node_stiffness[1]);
  EXPECT_EQ(expected.node_rotation[0],expected.node_rotation[1]);
  EXPECT_EQ(expected.node_stiffness[2],0);
  EXPECT_EQ(expected.node_rotation[2],0);
  EXPECT_EQ(expected.part_mass,2*expected.values[20]);
}
} // namespace beam18_test
