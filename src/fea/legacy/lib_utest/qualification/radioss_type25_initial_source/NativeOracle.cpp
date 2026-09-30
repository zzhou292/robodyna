// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <algorithm>
#include <climits>
#include <cmath>
#include <mutex>
#include <stdexcept>
extern "C" void initial_inventory_native(const int*,const double*,const int*,const int*,const int*,const int*,
    const int*,const int*,const double*,const double*,const double*,const double*,double*,const int*,
    const int*,const int*,const int*,const int*,const int*,const int*,const double*,int*,int*,int*,double*);
extern "C" void initial_prepared_native(const int*,int*);
extern "C" void rd_search_startup(const int*,const double*,const int*,const int*,const int*,
    const double*,const double*,const double*,double*,double*,int*,int*,int*,int*,int*);
namespace initial_source_test {
namespace {
void Require(bool yes){if(!yes)throw std::invalid_argument("Independent initial inventory packet rejected");}
bool Finite(double value){return std::isfinite(value);}
std::mutex native_mutex;
template<class T>const T* Data(const std::vector<T>& v,const T& empty){return v.empty()?&empty:v.data();}
}
InventoryResult NativeInventory(const InventoryInput& in) {
  const auto n=in.positions.size(),g=in.mains.size(),s=in.secondary.size(),ns=in.solids.size();
  Require(n&&n<=512&&g&&g<=320&&s&&s<=256&&ns<=64);
  Require(!in.main_nodes.empty()&&in.main_nodes.size()<=n&&in.types.size()==g&&in.codes.size()==n&&in.skews.size()==n&&
      in.coefficients.size()==g&&in.secondary_coefficients.size()==s&&in.main_gap.size()==g&&in.secondary_gap.size()==s&&
      in.corner_gaps.size()==g&&in.support.size()==g&&in.removed_nodes.size()==g&&Finite(in.multiplier)&&
      in.multiplier>0&&Finite(in.global_gap)&&in.global_gap>=0);
  const auto node=[&](int id){Require(id>0&&std::size_t(id)<=n);};
  for(const auto& point:in.positions)for(double value:point)Require(Finite(value));
  std::vector<int> native_codes(n);
  for(std::size_t i=0;i<n;++i) {
    Require(in.codes[i]>=0&&in.codes[i]<=7&&in.skews[i]>=0);
    native_codes[i]=512*in.codes[i]; // Literal COR3T consumes ICODE/512.
  }
  for(auto id:in.main_nodes)node(id);
  for(auto id:in.secondary)node(id);
  for(std::size_t i=0;i<s;++i)Require(Finite(in.secondary_coefficients[i])&&in.secondary_coefficients[i]>=0&&
      Finite(in.secondary_gap[i])&&in.secondary_gap[i]>=0);
  std::vector<int> offsets(g+1),removals;
  for(std::size_t i=0;i<g;++i) {
    for(auto id:in.mains[i])node(id);
    Require(Finite(in.coefficients[i])&&Finite(in.main_gap[i])&&in.main_gap[i]>=0);
    for(double gap:in.corner_gaps[i])Require(Finite(gap)&&gap>=0);
    Require(in.support[i][0]>=1&&std::size_t(in.support[i][0])<=ns+1&&in.support[i][1]>=0&&std::size_t(in.support[i][1])<=ns);
    for(auto id:in.removed_nodes[i]){node(id);removals.push_back(id);}
    Require(removals.size()<=n*g);offsets[i+1]=int(removals.size());
  }
  std::vector<int> solid_table(11*ns),parts(ns),node_offsets(n+1),node_incidence(8*ns),part_ids;
  for(const auto& solid:in.solids) {
    Require(solid.source_id>0&&solid.part_id>0);part_ids.push_back(solid.part_id);
    for(auto id:solid.nodes){node(id);++node_offsets[std::size_t(id)];}
  }
  std::sort(part_ids.begin(),part_ids.end());part_ids.erase(std::unique(part_ids.begin(),part_ids.end()),part_ids.end());
  for(std::size_t i=0;i<n;++i)node_offsets[i+1]+=node_offsets[i];
  auto cursor=node_offsets;
  for(std::size_t i=0;i<ns;++i) {
    const auto& solid=in.solids[i];parts[i]=int(std::lower_bound(part_ids.begin(),part_ids.end(),solid.part_id)-part_ids.begin()+1);
    solid_table[11*i+10]=solid.source_id;
    for(unsigned k=0;k<8;++k) {
      solid_table[11*i+1+k]=solid.nodes[k];
    }
  }
  for(unsigned k=0;k<8;++k)for(std::size_t i=0;i<ns;++i)
    node_incidence[std::size_t(cursor[std::size_t(in.solids[i].nodes[k]-1)]++)]=int(i+1);
  std::array<int,6> counts{int(n),int(g),int(s),int(in.main_nodes.size()),int(ns),int(removals.size())};
  std::array<double,3> controls{in.multiplier,in.global_gap,0};
  for(auto gap:in.secondary_gap)controls[2]=std::max(controls[2],gap);
  std::vector<int> cn(g*s),ce(g*s);int found=0,empty=0;
  InventoryResult out;out.corner_gaps=in.corner_gaps;std::array<double,5> diagnostics{};
  const std::lock_guard<std::mutex> lock(native_mutex);
  initial_inventory_native(counts.data(),in.positions.front().data(),in.mains.front().data(),in.types.data(),
      in.secondary.data(),in.main_nodes.data(),native_codes.data(),in.skews.data(),in.coefficients.data(),in.secondary_coefficients.data(),
      in.main_gap.data(),in.secondary_gap.data(),out.corner_gaps.front().data(),in.support.front().data(),Data(solid_table,empty),
      Data(parts,empty),node_offsets.data(),Data(node_incidence,empty),offsets.data(),Data(removals,empty),controls.data(),
      &found,cn.data(),ce.data(),diagnostics.data());
  if(found<0||std::size_t(found)>g*s)throw std::runtime_error("Native initial inventory returned invalid count");
  for(int i=0;i<found;++i) {
    if(cn[i]<1||std::size_t(cn[i])>s||ce[i]<1||std::size_t(ce[i])>g)
      throw std::runtime_error("Native initial inventory returned invalid identity");
    out.pairs.push_back({cn[i],ce[i]});
  }
  out.engine_margin=diagnostics[0];out.distance=diagnostics[1];out.zone=diagnostics[2];out.maximum_box=diagnostics[3];
  Require(Finite(diagnostics[4])&&diagnostics[4]>=0&&diagnostics[4]<=8000000);
  out.initialized_voxel_slots=std::uint64_t(diagnostics[4]);
  return out;
}
void NativePreparedMain(std::size_t mains,std::vector<std::array<int,4>>& rows) {
  Require(mains&&mains<=320&&!rows.empty()&&rows.size()<=256);
  for(const auto& row:rows)Require(row[2]>=0&&std::size_t(row[2])<=mains);
  const int counts[]{int(mains),int(rows.size())};const std::lock_guard<std::mutex> lock(native_mutex);
  initial_prepared_native(counts,rows.front().data());
}
type25_search_startup_test::NativeResult NativeGeometry(const InventoryInput& in,std::size_t primaries) {
  const auto n=in.positions.size(),g=in.mains.size(),s=in.secondary.size();
  Require(n&&n<=512&&g&&g<=320&&s&&s<=256&&primaries&&primaries<=g&&
      in.types.size()==g&&in.main_gap.size()==g&&in.secondary_gap.size()==s&&in.secondary_coefficients.size()==s);
  for(const auto& x:in.positions)for(double value:x)Require(Finite(value));
  for(const auto& main:in.mains)for(int node:main)Require(node>0&&std::size_t(node)<=n);
  for(int node:in.secondary)Require(node>0&&std::size_t(node)<=n);
  for(double value:in.main_gap)Require(Finite(value)&&value>=0);
  for(double value:in.secondary_gap)Require(Finite(value)&&value>=0);
  for(double value:in.secondary_coefficients)Require(Finite(value)&&value>=0);
  const int counts[]{int(n),int(g),int(s),int(primaries),0};
  std::vector<int> offsets(g+1),nodes(g*s),inverse(s+1),mains(g*s);
  type25_search_startup_test::NativeResult out;out.extent.resize(primaries);out.contact.resize(s);
  const std::lock_guard<std::mutex> lock(native_mutex);
  rd_search_startup(counts,in.positions.front().data(),in.mains.front().data(),in.types.data(),in.secondary.data(),
      in.secondary_gap.data(),in.secondary_coefficients.data(),in.main_gap.data(),out.scalar.data(),out.extent.data(),
      offsets.data(),nodes.data(),inverse.data(),mains.data(),out.contact.data());
  Require(offsets.back()>=0&&std::size_t(offsets.back())<=g*s&&offsets.back()==inverse.back());
  out.main_offsets.assign(offsets.begin(),offsets.end());out.secondary_offsets.assign(inverse.begin(),inverse.end());
  for(int i=0;i<offsets.back();++i){Require(nodes[i]>0&&std::size_t(nodes[i])<=n);out.removed_nodes.push_back(std::uint32_t(nodes[i]-1));
    Require(mains[i]>0&&std::size_t(mains[i])<=g);out.removed_mains.push_back(std::uint32_t(mains[i]));}
  return out;
}
}
