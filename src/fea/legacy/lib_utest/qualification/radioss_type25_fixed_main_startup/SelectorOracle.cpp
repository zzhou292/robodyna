// SPDX-License-Identifier: AGPL-3.0-or-later
#include "SelectorOracle.h"
#include <climits>
namespace type25_startup_test {
extern "C" void rd_startup_selector(const int*,const double*,const int*,const int*,const int*,const int*,const int*,int*,double*,double*,double*);
NativeSelection SelectorOracle(const Case& source,int self,unsigned edge,const std::vector<int>& candidates) {
  if(source.ids.empty()||source.ids.size()>256||source.primary.empty()||source.primary.size()>160||
      candidates.size()<2||candidates.size()>160||self<1||std::size_t(self)>source.primary.size()||edge>3)
    throw std::invalid_argument("Native selector fixture bound");
  std::vector<int> faces;std::vector<double> positions=source.positions;
  if(positions.size()!=3*source.ids.size())throw std::invalid_argument("Native selector position extent");
  if(source.units==s::Coordinates::Si) {
    if(!std::isfinite(source.scale.length_m)||source.scale.length_m<=0)throw std::invalid_argument("Native selector units");
    for(auto& x:positions)x/=source.scale.length_m;
  } else if(source.units!=s::Coordinates::Native)throw std::invalid_argument("Native selector coordinate mode");
  for(const auto x:positions)if(!std::isfinite(x))throw std::invalid_argument("Native selector coordinate");
  for(const auto& face:source.primary)for(auto node:face.nodes) {
    if(node>=source.ids.size())throw std::invalid_argument("Native selector connectivity");
    faces.push_back(int(node+1));
  }
  for(auto id:candidates)if(id<1||std::size_t(id)>source.primary.size()||id==self)throw std::invalid_argument("Native selector candidate");
  const int counts[]{int(source.ids.size()),int(source.primary.size()),int(candidates.size())};
  const int i1=int(source.primary[self-1].nodes[edge]+1),i2=int(source.primary[self-1].nodes[(edge+1)%4]+1);
  int result[3]{};NativeSelection out;out.angles.resize(candidates.size());out.sides.resize(candidates.size());
  rd_startup_selector(counts,positions.data(),faces.data(),candidates.data(),&self,&i1,&i2,result,out.angles.data(),out.sides.data(),&out.em20);
  out.winner=result[0];out.warning=result[1];out.calls=result[2];return out;
}
} // namespace type25_startup_test
