// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TiedClassificationInternal.h"

namespace tl::constraints::tied_shell::classification_detail {
namespace {
// KINSET's six-direction branch, with ISK=0 for both selected callers. The
// packed skew is retained when another direction is added to an existing node.
bool AddDirection(int direction,std::int32_t& packed) noexcept {
  const auto skew=packed/10;
  const auto mask=packed-10*skew;
  const auto bit=1<<(direction%3);
  if(mask==0) {
    packed=bit;
    return false;
  }
  if((mask&bit)==0) {
    packed+=bit;
    return skew!=0;
  }
  return true;
}
}
bool KinSet(int kind,int direction,NativeKinematics& node,NativeKinematics& scratch,
            std::array<std::int32_t,8192>& itf,std::uint64_t& warnings) noexcept {
  if(direction<0 || direction>=6 || (kind!=2 && kind!=8 && kind!=128) ||
     !Mask(node.conditions) || !Mask(node.duplicate_conditions) ||
     !Mask(node.incompatible_conditions) || !Directions(node.translation) ||
     !Directions(node.rotation)) return false;
  const bool rotational=direction>=3;
  const auto warn=AddDirection(direction,rotational ? node.rotation : node.translation);
  const auto duplicate=AddDirection(direction,rotational ? scratch.rotation : scratch.translation);
  if(duplicate && Decode(kind,node.conditions,itf)==1 && Decode(kind,node.duplicate_conditions,itf)==0)
    node.duplicate_conditions+=kind;
  if(warn && Decode(kind,node.conditions,itf)==0 && Decode(kind,node.incompatible_conditions,itf)==0)
    node.incompatible_conditions+=kind;
  if(Decode(kind,node.conditions,itf)==0) node.conditions+=kind;
  if(!Mask(node.conditions) || !Mask(node.duplicate_conditions) || !Mask(node.incompatible_conditions))
    return false;
  // Native KWARN suppresses the simultaneous IWL/IRB condition specifically;
  // it does not suppress IRB2 or every wall-related duplicate.
  if(!(Decode(4,node.conditions,itf)==1 && Decode(8,node.conditions,itf)==1))
    warnings+=warn;
  return true;
}
} // namespace tl::constraints::tied_shell::classification_detail
