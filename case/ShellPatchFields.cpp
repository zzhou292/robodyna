#include "ShellPatchFields.h"
#include <cmath>

namespace crash::case_data {
using namespace crash::output;
void AppendShellElements(Document& doc,const tl::fea::reissner::ShellResult* source_elements,
                         std::size_t count,const std::uint64_t* parent_ids) {
    Require(source_elements&&parent_ids&&count>0&&count<=reference::kCouponElements,"Invalid shell field table");
    Value elements(rapidjson::kArrayType);
    for(unsigned e=0;e<count;++e) {
        const auto& source=source_elements[e]; Value element(rapidjson::kObjectType);
        Require(std::isfinite(source.energy)&&std::isfinite(source.bending_energy),"Nonfinite coupon element energy");
        element.AddMember("parent_element",Value().SetUint64(parent_ids[e]),doc.GetAllocator());
        element.AddMember("elastic_energy_J",source.energy,doc.GetAllocator());
        element.AddMember("bending_energy_J",source.bending_energy,doc.GetAllocator());
        Value forces(rapidjson::kArrayType),couples(rapidjson::kArrayType);
        for(unsigned n=0;n<4;++n) {
            const double force[]{source.force[n].x,source.force[n].y,source.force[n].z};
            const double couple[]{source.couple[n].x,source.couple[n].y,source.couple[n].z};
            forces.PushBack(FiniteArray(doc,force,3),doc.GetAllocator()); couples.PushBack(FiniteArray(doc,couple,3),doc.GetAllocator());
        }
        element.AddMember("force_world_N",forces,doc.GetAllocator()); element.AddMember("couple_world_Nm",couples,doc.GetAllocator());
        Value strain(rapidjson::kArrayType),resultant(rapidjson::kArrayType);
        for(unsigned p=0;p<4;++p) {
            strain.PushBack(FiniteArray(doc,source.strain[p],12),doc.GetAllocator());
            resultant.PushBack(FiniteArray(doc,source.resultant[p],12),doc.GetAllocator());
        }
        element.AddMember("gauss_strain",strain,doc.GetAllocator()); element.AddMember("gauss_resultant",resultant,doc.GetAllocator());
        elements.PushBack(element,doc.GetAllocator());
    }
    doc.AddMember("elements",elements,doc.GetAllocator());
}
void AppendShellReference(Document& doc,const reference::ElasticCouponData& model) {
    Value nodes(rapidjson::kArrayType),sections(rapidjson::kArrayType);
    for(unsigned n=0;n<reference::kCouponNodes;++n) {
        Value node(rapidjson::kObjectType);
        const auto& mass=model.nodal_mass[n];
        node.AddMember("mass_kg",mass.mass,doc.GetAllocator());
        node.AddMember("physical_tangential_inertia_kg_m2",mass.physical_tangential_inertia,doc.GetAllocator());
        node.AddMember("artificial_drilling_inertia_kg_m2",mass.artificial_drilling_inertia,doc.GetAllocator());
        node.AddMember("clamped",model.fixed[n],doc.GetAllocator());
        const auto& p=model.reference_configuration.position[n]; const double position[]{p.x,p.y,p.z};
        const auto& q=model.reference_configuration.rotation[n]; const double rotation[]{q.w,q.x,q.y,q.z};
        node.AddMember("reference_position_m",FiniteArray(doc,position,3),doc.GetAllocator());
        node.AddMember("reference_orientation_wxyz",FiniteArray(doc,rotation,4),doc.GetAllocator());
        nodes.PushBack(node,doc.GetAllocator());
    }
    for(unsigned e=0;e<reference::kCouponElements;++e) {
        Value element(rapidjson::kObjectType),connectivity(rapidjson::kArrayType);
        for(auto node:model.connectivity[e])connectivity.PushBack(Value().SetUint64(node),doc.GetAllocator());
        element.AddMember("connectivity_zero_based",connectivity,doc.GetAllocator());
        element.AddMember("section_stiffness_12x12_row_major",FiniteArray(doc,model.section[e].stiffness,144),doc.GetAllocator());
        sections.PushBack(element,doc.GetAllocator());
    }
    doc.AddMember("reference_nodes",nodes,doc.GetAllocator()); doc.AddMember("reference_elements",sections,doc.GetAllocator());
}
void AppendSurfaceBinding(Document& doc,const visual::Binding& b) {
    Value vertices(rapidjson::kArrayType),triangles(rapidjson::kArrayType);
    for(const auto& v:b.vertices) {
        Value item(rapidjson::kArrayType);
        for(std::uint64_t n:{std::uint64_t(v.tl_node),v.source.asset,v.source.instance,v.source.node})item.PushBack(Value().SetUint64(n),doc.GetAllocator());
        vertices.PushBack(item,doc.GetAllocator());
    }
    for(const auto& t:b.triangles) {
        Value item(rapidjson::kArrayType);
        for(std::uint64_t n:{std::uint64_t(t.vertices[0]),std::uint64_t(t.vertices[1]),std::uint64_t(t.vertices[2]),
                            t.asset,t.instance,t.element,t.part,std::uint64_t(t.local_face),std::uint64_t(t.subtriangle)})
            item.PushBack(Value().SetUint64(n),doc.GetAllocator());
        triangles.PushBack(item,doc.GetAllocator());
    }
    String(doc,"vertex_binding_columns","tl_node,asset,instance,source_node");
    String(doc,"triangle_binding_columns","v0,v1,v2,asset,instance,parent_element,part,local_face,subtriangle");
    doc.AddMember("vertex_binding",vertices,doc.GetAllocator()); doc.AddMember("triangle_binding",triangles,doc.GetAllocator());
}
} // namespace crash::case_data
