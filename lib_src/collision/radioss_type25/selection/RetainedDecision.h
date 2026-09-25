// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "RetainedBoundary.h"
#include "ClassificationValues.h"
namespace tlfea::contact::radioss_type25::selection::detail {
TL_MATH_HOST_DEVICE inline void SelectRetained(const NativePairInput&,const Work& w,
    NativeRetainedResult& out) {
  int selected=SelectClosest(w,out,out.prior_subtriangle);
  if(selected!=0&&out.sector[selected-1].penetration==0)selected=0;
  for(unsigned i=0;i<4;++i)if(int(i+1)!=selected)out.sector[i].penetration=0;
  out.selected_subtriangle=selected;
  auto& row=out.history.row;
  row.irtlm[1]=selected+5*out.prior_subtriangle;
  row.selection_metric[0]=out.distance_squared;
  const unsigned old=unsigned(out.prior_subtriangle-1);
  const bool slide=out.sector[old].penetration==0 ||
      (!w.frame.triangle ? out.sector[old].far>=2 :
       out.sector[0].far>=2||out.sector[1].far>=2||out.sector[2].far>=2);
  if(slide)row.irtlm[1]=-row.irtlm[1];
}
} // namespace tlfea::contact::radioss_type25::selection::detail
