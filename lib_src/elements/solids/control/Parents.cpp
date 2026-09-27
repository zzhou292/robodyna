// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
namespace tl::fea::solids::control::detail {
Report BindParents(const SolidNodeContributions& coefficients,Input in,Storage& storage,
                    Scratch& scratch,std::size_t& controlled) noexcept {
  const auto rows=coefficients.parents();std::size_t local[5]{};
  for(std::size_t i=0;i<rows.size();++i) {
    const auto family=static_cast<unsigned>(rows[i].family);
    if(family>=5)return {Status::UnsupportedProfile,"Unknown model solid family",i};
    storage.parents[i].family=rows[i].family;storage.parents[i].family_index=local[family]++;
  }
  for(std::size_t i=0;i<in.parents.size();++i) {
    const auto& source=in.parents[i];const auto at=scratch.identities.First(source.element_id);
    if(at==SIZE_MAX)return {Status::SourceMismatch,"Control row element is absent from complete model"};
    if(scratch.seen[at])return {Status::DuplicateIdentity,"Repeated control source element",at};
    const auto& row=rows[at];
    if(source.part_id!=row.source_part_id||source.section_id!=row.source_section_id||source.material_id!=row.source_material_id)
      return {Status::SourceMismatch,"Control source PID/SID/MID differs from model",at};
    if(!source.native_property_id||source.icontrol>1)
      return {Status::InvalidInput,"Native property and selected ICONTROL0/1 required",at};
    if(source.icontrol&&!Supported(row.family))return {Status::UnsupportedProfile,"No selected native control implementation for family",at};
    storage.parents[at].source=source;scratch.seen[at]=1;controlled+=source.icontrol?1:0;
  }
  for(std::size_t i=0;i<rows.size();++i) {
    if(!scratch.seen[i])return {Status::SourceMismatch,"Control source row omitted model element",i};
    scratch.seen[i]=0;
  }
  return {};
}
} // namespace tl::fea::solids::control::detail
