#include "ComponentContext.h"
#include <cstring>

namespace crash::output::assembly::binary::detail {
void StageFrame(const records::Context& context,const wall_fields::FrameView& view,
                records::FrameRecord& output) {
    wall_fields::CheckFrame(view);
    if(view.stamp->epoch)wall_fields::CheckContactPhase(view);
    const auto& s=*view.stamp;const auto& id=context.identity();
    const auto& map=view.surface->binding();const auto& source=view.bindings->source().data();
    const auto& settings=*view.setup->settings();
    Require(id.owner==s.owner_id&&id.run==map.identity.run&&id.topology==map.identity.topology&&
        id.source_instance==view.bindings->source_instance_id()&&id.source_inventory_bytes==source.identity.bytes&&
        id.source_inventory_sha256==source.identity.sha256&&id.configuration==settings.configuration_id&&
        id.qualification==settings.qualification_id&&Bits(context.fixed_dt())==Bits(s.fixed_dt)&&
        context.nodes()==s.node_count&&context.parents().size()==view.surface->parents().size()&&
        output.position_xyz.size()==3*context.nodes()&&output.plastic_points.size()==context.points(),
        "Component frame context/active output extent mismatch");
    records::FrameStamp stamp{s.epoch,s.reaction_base_epoch,view.captured_shells->qeph.attempt,
        s.time,s.reaction_time,s.velocity_time,s.reaction_kick_dt};
    records::CheckStamp(context,stamp);
    for(const auto& p:view.surface->parents()) {
        const auto& declared=context.parents()[p.source_index];
        Require(declared.source_element==p.element&&declared.source_part==p.part&&
            declared.source_elform==p.source_elform&&declared.native_family==
                (p.family==source::ShellFamily::Qeph?QephFamily:T3Family)&&declared.native_points==3&&
            declared.plastic==records::PlasticField::NativeEquivalentPlasticStrain&&
            context.point_offsets()[p.source_index+1]-context.point_offsets()[p.source_index]==3,
            "Component source/native point layout changed");
    }
    // ValidSections above validates every native point before these infallible
    // copies. No max reduction, decimal conversion or quadrature reordering.
    std::memcpy(output.position_xyz.data(),view.nodes.position_xyz,output.position_xyz.size()*sizeof(double));
    for(const auto& p:view.surface->parents()) {
        const auto& section=(p.family==source::ShellFamily::Qeph?view.qeph:view.t3).values[p.family_index];
        for(std::size_t point=0;point<3;++point)
            output.plastic_points[context.point_offsets()[p.source_index]+point]=section.history.point[point].plastic_strain;
    }
    output.stamp=stamp;
}
} // namespace crash::output::assembly::binary::detail
