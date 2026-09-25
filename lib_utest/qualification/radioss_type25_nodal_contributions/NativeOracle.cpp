// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <stdexcept>
namespace type25_contribution_test {
extern "C" void rd_nodal_bulk(const double*,const int*,const double*,double*,double*);
extern "C" void rd_nodal_spring(const double*,const int*,const double*,double*);
extern "C" void rd_nodal_length(const double*,const int*,const double*,double*,int*);
namespace {
int Property(n::SpringNodalKind kind) {
  if(kind==n::SpringNodalKind::Type13)return 13;
  if(kind==n::SpringNodalKind::Type25)return 25;
  throw std::invalid_argument("Unselected native spring property");
}
}
NativeSolidObservation OracleSolid(const n::NativeSolidNodalInput& in,double seed) {
  if(in.kind!=n::SolidNodalKind::Hex8&&in.kind!=n::SolidNodalKind::Penta6)throw std::invalid_argument("Unselected native solid shape");
  const int kind=in.kind==n::SolidNodalKind::Hex8?8:6;const double values[]{in.volume,in.fill,in.bulk};
  double rows[16],extra[24];rd_nodal_bulk(values,&kind,&seed,rows,extra);NativeSolidObservation out;
  for(unsigned i=0;i<8;++i){out.volume[i]=rows[i];out.bulk_volume[i]=rows[8+i];}
  for(unsigned i=0;i<12;++i){out.extended_volume[i]=extra[i];out.extended_bulk_volume[i]=extra[12+i];}
  return out;
}
double OracleSpringPrepared(const n::NativeSpringNodalInput& in,double xl,double seed) {
  const int flags[]{Property(in.kind),in.interface_initialization};double values[7]{};
  if(in.interface_initialization) {
    const unsigned count=in.kind==n::SpringNodalKind::Type13?3:2;
    for(unsigned i=0;i<count;++i){values[2*i]=in.translation[i].slope;values[2*i+1]=in.translation[i].scale;}
    values[6]=xl;
  }
  double out;rd_nodal_spring(values,flags,&seed,&out);return out;
}
NativeLengthObservation OracleLength(n::SpringNodalKind kind,int mode,const std::array<double,6>& points,double noise) {
  const int flags[]{Property(kind),mode};NativeLengthObservation out;
  rd_nodal_length(points.data(),flags,&noise,&out.value,&out.diagnostics);return out;
}
} // namespace type25_contribution_test
