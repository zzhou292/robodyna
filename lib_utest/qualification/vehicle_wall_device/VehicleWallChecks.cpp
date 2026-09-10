#include "VehicleWallFixture.h"
namespace vehicle_wall_device_test {
void CheckIncidence(const Fixture& f,const detail::PreparedModel& prepared) {
  const auto& m=prepared.model();ASSERT_EQ(m.node_count,f.n);ASSERT_EQ(m.parent_count,f.nq+f.nt);
  ASSERT_EQ(m.incident_offsets[0],0u);ASSERT_EQ(m.incident_offsets[f.n],4*f.nq+3*f.nt);
  std::vector<bool> seen(4*m.parent_count);
  for(unsigned i=0;i<m.node_count;++i) {
    ASSERT_EQ(m.nodes[i].node,i);ASSERT_LT(m.incident_offsets[i],m.incident_offsets[i+1]);
    ASSERT_LE(m.incident_offsets[i+1],4*f.nq+3*f.nt);
    for(unsigned j=m.incident_offsets[i];j<m.incident_offsets[i+1];++j) {
      const auto slot=m.incident_slots[j],p=slot/4,l=slot%4;
      ASSERT_LT(p,m.parent_count);ASSERT_LT(l,m.parents[p].arity);EXPECT_EQ(m.parents[p].nodes[l],i);
      ASSERT_FALSE(seen[slot]);seen[slot]=true;
      if(j>m.incident_offsets[i])EXPECT_LT(m.incident_slots[j-1],slot);
    }
  }
  for(unsigned p=0;p<m.parent_count;++p)for(unsigned l=0;l<4;++l)EXPECT_EQ(seen[4*p+l],l<m.parents[p].arity);
}
void CheckLoads(const Fixture& f,const Results& result,const std::vector<double>& x) {
  long double force=0,energy=0;
  for(std::size_t n=0;n<f.n;++n) {
    const long double depth=std::max(0.L,static_cast<long double>(x[3*n]));
    const long double expected=16*f.area[n]*depth,potential=expected*depth/2;
    const auto& node=result.nodes[n];ASSERT_EQ(node.node,n);ASSERT_TRUE(node.valid);
    EXPECT_LE(node.force.lower,expected);EXPECT_GE(node.force.upper,expected);
    EXPECT_LE(node.potential.lower,potential);EXPECT_GE(node.potential.upper,potential);
    EXPECT_NE(result.faces[n],0u);force+=expected;energy+=potential;
  }
  for(std::size_t p=0;p<f.nq+f.nt;++p) {
    const auto& actual=result.parents[p];const auto& source=f.weights.parent(p);
    ASSERT_TRUE(actual.valid);ASSERT_EQ(actual.parent_element_id,source.parent_element_id);
    EXPECT_EQ(actual.feature_id,source.feature_id);ASSERT_EQ(actual.arity,source.arity);
    const long double area=source.arity==4?1.L/16:1.L/24;
    for(unsigned l=0;l<actual.arity;++l) {
      const long double term=16*area*std::max(0.L,static_cast<long double>(x[3*source.nodes[l]]));
      EXPECT_LE(actual.force[l].lower,term);EXPECT_GE(actual.force[l].upper,term);
    }
    if(source.arity==3)EXPECT_EQ(actual.force[3].value,0);
  }
  EXPECT_LE(result.diagnostics.resultant.lower,force);EXPECT_GE(result.diagnostics.resultant.upper,force);
  EXPECT_LE(result.diagnostics.potential.lower,energy);EXPECT_GE(result.diagnostics.potential.upper,energy);
  EXPECT_EQ(result.diagnostics.parent_count,f.nq+f.nt);EXPECT_EQ(result.diagnostics.node_count,f.n);
}
void SameResults(const Results& a,const Results& b) {
  EXPECT_EQ(nodal_wall_owner_test::Bytes(a.diagnostics),nodal_wall_owner_test::Bytes(b.diagnostics));
  EXPECT_EQ(std::memcmp(a.parents.data(),b.parents.data(),a.parents.size()*sizeof(a.parents[0])),0);
  EXPECT_EQ(std::memcmp(a.nodes.data(),b.nodes.data(),a.nodes.size()*sizeof(a.nodes[0])),0);EXPECT_EQ(a.faces,b.faces);
}
std::vector<double> Forces(const fe::NodalAssemblyView& a) {
  const auto n=a.accepted.node_count;std::vector<double> result(6*n);
  const double* fields[]{a.forces.force_x,a.forces.force_y,a.forces.force_z,a.forces.couple_x,a.forces.couple_y,a.forces.couple_z};
  for(unsigned i=0;i<6;++i)EXPECT_EQ(cudaMemcpyAsync(result.data()+i*n,fields[i],n*sizeof(double),cudaMemcpyDeviceToHost,a.stream),cudaSuccess);
  EXPECT_EQ(cudaStreamSynchronize(a.stream),cudaSuccess);return result;
}
} // namespace vehicle_wall_device_test
