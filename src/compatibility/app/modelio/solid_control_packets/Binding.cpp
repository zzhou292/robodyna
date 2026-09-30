#include "Internal.h"
#include <algorithm>
namespace crash::modelio::solid_control_packets::detail {
control::Family Family(solid_source::Family family) {
    switch(family) {
        case solid_source::Family::Solid18:return control::Family::Solid18;
        case solid_source::Family::Solid24:return control::Family::Solid24;
        case solid_source::Family::Solid6z:return control::Family::Solid6z;
        case solid_source::Family::Solid18Law44:return control::Family::Solid18Law44;
        case solid_source::Family::Solid18Law90:return control::Family::Solid18Law90;
    }
    throw std::runtime_error("Unknown solid source family");
}
void Bind(const Values& input,const solid_source::Data& solids,const solid_control::EffectiveData& controls) {
    Require(input.parents.size()==solids.rows.size()&&input.parent_families.size()==input.parents.size(),
        "Native packet export and selected solid population differ");
    for(std::size_t i=0;i<controls.parts.size();++i)
        Require(controls.parts[i].part_id&&(i==0||controls.parts[i-1].part_id<controls.parts[i].part_id),
            "Effective solid control authority has unordered or duplicate parts");
    tl::util::SourceIdentityIndex<0> index;
    index.Prepare(solids.rows.size(),[&](std::size_t i){return solids.rows[i].element_id;});
    std::vector<std::uint8_t> seen(solids.rows.size(),0);
    for(std::size_t i=0;i<input.parents.size();++i) {
        const auto& p=input.parents[i];const auto row_index=index.First(p.element_id);
        Require(row_index<solids.rows.size()&&!seen[row_index],"Native parent is absent or duplicated");seen[row_index]=1;
        const auto& row=solids.rows[row_index];Require(row.part_index<solids.parts.size(),"Solid material association is invalid");
        const auto& part=solids.parts[row.part_index];
        Require(p.part_id==row.part_id&&p.part_id==part.id&&p.section_id==part.section_id&&
            p.material_id==part.material_id&&input.parent_families[i]==Family(row.family),
            "Native parent identity, material or formulation differs from prepared solids");
        const auto found=std::lower_bound(controls.parts.begin(),controls.parts.end(),p.part_id,
            [](const auto& value,std::uint64_t id){return value.part_id<id;});
        Require(found!=controls.parts.end()&&found->part_id==p.part_id&&found->section_id==p.section_id&&
            found->material_id==p.material_id&&p.icontrol==unsigned(found->effective_control),
            "Native ICONTROL differs from independent source property authority");
        // Unrequested multi-MID clones may have no direct native property ID.
        // In that case keep the authenticated observed mapping; never invent one.
        Require(found->native_property_id?found->native_property_id==p.native_property_id:!p.icontrol,
            "Native property identity differs from available source mapping");
    }
}
} // namespace crash::modelio::solid_control_packets::detail
