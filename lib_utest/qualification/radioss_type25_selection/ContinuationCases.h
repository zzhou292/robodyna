// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Cases.h"
namespace type25_selection_test {
struct ContinuationCase {
  std::string name;
  s::NativeContinuationInput input;
  n::NativeGeometryHistory prior;
};
inline ContinuationCase Continue(const Case& source) {
  ContinuationCase out;out.name=source.name;out.input.pair=source.input;out.prior=source.prior;
  out.input.segment_count=16;
  out.prior.row.irtlm[0]=2;out.prior.row.irtlm[2]=8;
  out.prior.row.selection_metric[0]=out.prior.row.selection_metric[1]=1e20;
  for(unsigned i=0;i<4;++i)
    out.input.normal_reference[i]=out.input.pair.boundary_ids[i]?
        int(out.input.pair.boundary_ids[i]):int(10000+i);
  return out;
}
inline ContinuationCase BasicContinuation() { return Continue(Basic()); }
inline std::vector<ContinuationCase> ContinuationCases() {
  std::vector<ContinuationCase> cases;
  for(const auto& c:Cases())cases.push_back(Continue(c));
  // Same geometry, all native reference-match and solid/coating symmetry routes.
  for(bool triangle:{false,true})for(int role:{0,4,17,-4,-17})for(int skew:{0,1,2}) {
    auto c=Continue(FromGeometry({"symmetry",triangle?type25_geometry_test::Triangle():type25_geometry_test::Quad()}));
    c.input.pair.segment_type=role;c.input.secondary_skew=skew;
    c.input.secondary_constraint=1;
    for(unsigned i=0;i<4;++i)c.input.main_constraint[i]=1;
    cases.push_back(c);
  }
  for(unsigned match=0;match<5;++match)for(bool triangle:{false,true}) {
    auto c=Continue(FromGeometry({"sliding",triangle?type25_geometry_test::Triangle():type25_geometry_test::Quad()}));
    c.input.pair.secondary={-.5,-.5,-.2};
    if(match<4)c.input.sliding_reference[0]=c.input.normal_reference[match];
    cases.push_back(c);
  }
  return cases;
}
} // namespace type25_selection_test
