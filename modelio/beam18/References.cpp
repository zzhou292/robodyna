#include "Internal.h"
#include "lib_src/elements/beam18/Reference.h"
namespace crash::modelio::beam18::detail {
void PrepareReferences(Data& data) {
    for(auto& row:data.rows) {
        const auto& part=data.parts.at(row.part_index);
        native::Input input;input.source_element_id=row.element_id;input.source_part_id=row.part_id;
        input.source_section_id=part.section_id;input.source_material_id=part.material_id;
        input.units=native::WorkingUnits::TonneMillimetreSecond;input.profile=native::Profile::CircularFourPointStoredZero;
        input.radius=part.radius_mm;input.density=part.density_working;input.young=part.young_working;
        input.poisson=part.material.material.poisson_ratio;input.local=row.raw_record[9];
        for(unsigned k=0;k<4;++k)input.release[k]=row.raw_record[5+k];
        for(unsigned k=0;k<3;++k){const auto& n=data.nodes.at(row.nodes[k]);input.source_node_id[k]=n.id;input.position[k]=n.position_working;}
        const auto status=native::InitializeReference(input,row.reference);
        Require(status==native::Status::Success,"Original beam18 reference initialization failed");
        for(unsigned k=0;k<2;++k) {
            const auto& x=row.reference.geometry().endpoint_m[k];const auto& y=data.nodes.at(row.nodes[k]).position_m;
            Require(output::Bits(x.x)==output::Bits(y.x) && output::Bits(x.y)==output::Bits(y.y) && output::Bits(x.z)==output::Bits(y.z),
                "Beam18 initialized reference differs from canonical endpoint");
        }
    }
}
} // namespace crash::modelio::beam18::detail
