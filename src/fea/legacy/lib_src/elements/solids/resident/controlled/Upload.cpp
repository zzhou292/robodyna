// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Upload.h"
#include "lib_src/elements/solid24/controlled_distortion/Stage.h"
#include "lib_src/elements/solid18/total_strain/controlled_distortion/Stage.h"
namespace tl::fea::solids::batch_detail::controlled {
BatchReport Upload(const Model& model,util::HostArena& arena,const ArenaLayout& layout,batch_detail::Storage& header) {
  const auto& l=layout.controlled;auto& out=header.controlled;
  if(!l.packets.count)return {};
  if((l.index24.count&&!arena.Construct<std::size_t>(l.index24))||(l.index90.count&&!arena.Construct<std::size_t>(l.index90))||
     (l.reference24.count&&!arena.Construct<h24::Reference>(l.reference24))||(l.reference90.count&&!arena.Construct<foam::Reference>(l.reference90))||
     (l.packets.count&&!arena.Construct<control::Packet>(l.packets))||(l.members.count&&!arena.Construct<control::Member>(l.members))||
     (l.workspace.count&&!arena.Construct<Workspace>(l.workspace)))return {BatchStatus::ResourceLimit,"Controlled solid arena construction failed"};
  for(std::size_t p=0;p<l.index24.count;++p)out.index24[p]=SIZE_MAX;
  for(std::size_t p=0;p<l.index90.count;++p)out.index90[p]=SIZE_MAX;
  const auto& selected=*model.control_selection();std::size_t at24=0,at90=0;
  for(const auto& row:selected.parents())if(row.source.icontrol) {
    if(row.family==Family::Solid24) {
      const auto& parent=model.solid24()[row.family_index];
      const auto status=h24::PrepareReference(parent.reference,model.materials42()[parent.material_index].value,
        selected.units(),out.reference24[at24]);
      if(status!=solid24::ForceStatus::Success)return {BatchStatus::InvalidInput,"Controlled H24 reference rejected",row.family,row.family_index};
      out.index24[row.family_index]=at24++;
    } else if(row.family==Family::Solid18Law90) {
      const auto& parent=model.solid18_law90()[row.family_index];foam::Material material;
      if(foam::PrepareMaterial(model.materials90()[parent.material_index].value,selected.units(),material)!=solid_common::distortion::Status::Success||
         foam::PrepareReference(parent.reference,material,out.reference90[at90])!=solid_common::distortion::Status::Success)
        return {BatchStatus::InvalidInput,"Controlled LAW90 reference rejected",row.family,row.family_index};
      out.index90[row.family_index]=at90++;
    } else return {BatchStatus::InvalidInput,"Unsupported controlled resident family",row.family,row.family_index};
  }
  if(at24!=out.count24||at90!=out.count90)return {BatchStatus::InvalidInput,"Controlled source extent changed during upload"};
  for(std::size_t p=0;p<out.packet_count;++p)out.packets[p]=selected.packets()[p];
  for(std::size_t p=0;p<out.member_count;++p)out.members[p]=selected.members()[p];
  return {};
}
BatchReport Relocate(const Model& model,batch_detail::Storage& header)noexcept {
  if(!header.controlled.index90)return {};
  for(std::size_t p=0;p<model.solid18_law90().size();++p) {
    const auto at=header.controlled.index90[p];if(at==SIZE_MAX)continue;
    if(at>=header.controlled.count90)return {BatchStatus::InvalidInput,"Controlled LAW90 reference index invalid"};
    const auto material=model.solid18_law90()[p].material_index;
    foam::Reference relocated;
    if(foam::RelocateReference(header.controlled.reference90[at],header.material90[material].curve(),relocated)!=solid_common::distortion::Status::Success)
      return {BatchStatus::InvalidInput,"Controlled LAW90 curve relocation rejected",Family::Solid18Law90,p};
    header.controlled.reference90[at]=relocated;
  }
  return {};
}
} // namespace tl::fea::solids::batch_detail::controlled
