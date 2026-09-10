#include "SourceAssemblyConnectorFields.h"
#include "WallFieldValues.h"

namespace crash::output::assembly::wall_fields {
Document ConnectorInputDocument(const cases::source_assembly::SourceAssemblyBindings& b) {
    const auto* model=b.connectors();const auto* declaration=b.spotweld_declaration();
    Require(model&&b.combined_mass()&&declaration&&model->connection_count()&&model->connection_count()<=MaxArchivedConnectors&&
        model->connection_count()==b.source().data().internal_spotwelds.size()&&
        b.combined_mass()->Matches(*model)&&b.combined_mass()->Matches(b.shells())&&
        declaration->policy==source::SpotweldPolicy::OpenRadiossTonneMillimetreSecondDirectImport,
        "Complete explicitly declared source connectors are required");
    Document d;d.SetObject();String(d,"kind",ConnectorKind);String(d,"policy",ConnectorPolicy);
    Integer(d,"source_instance_id",b.source_instance_id());Integer(d,"connection_count",model->connection_count());
    Integer(d,"property_count",model->property_count());const auto units=model->source_units();
    Put(d,"source_units_to_SI",Values(d,{units.mass_to_kg,units.length_to_m,units.time_to_s}));
    Value properties(rapidjson::kArrayType),connections(rapidjson::kArrayType),endpoints(rapidjson::kArrayType);
    for(std::size_t i=0;i<model->property_count();++i) {
        const auto& property=model->properties()[i];const auto& p=property.property;Document row;row.SetObject();
        Integer(row,"generated_property_id",property.source_property_id);Number(row,"mass_kg",p.mass_kg);
        Number(row,"isotropic_inertia_kg_m2",p.isotropic_inertia_kg_m2);
        FiniteArray(row,"stiffness",p.stiffness,4);FiniteArray(row,"damping",p.damping,4);
        FiniteArray(row,"failure_negative",p.failure_negative,4);FiniteArray(row,"failure_positive",p.failure_positive,4);
        FiniteArray(row,"failure_weight",p.failure_weight,4);FiniteArray(row,"failure_exponent",p.failure_exponent,4);
        Value value;value.CopyFrom(row,d.GetAllocator());properties.PushBack(value,d.GetAllocator());
    }
    for(std::size_t i=0;i<model->connection_count();++i) {
        const auto& c=model->connections()[i];const auto& r=model->references()[i];
        const auto& source=b.source().data().internal_spotwelds[i];
        Require(c.source_element_id==source.record.id&&source.record.cards.size()==2,"Connector source order changed");
        Document row;row.SetObject();Integer(row,"source_element_id",c.source_element_id);
        Integer(row,"generated_property_id",model->properties()[c.property_index].source_property_id);
        Put(row,"source_node_ids",Ids(row,{c.source_node_id[0],c.source_node_id[1]}));
        Put(row,"global_nodes",Ids(row,{c.global_node[0],c.global_node[1]}));
        Put(row,"reference_positions_m",Values(row,{r.position[0].x,r.position[0].y,r.position[0].z,
                                                    r.position[1].x,r.position[1].y,r.position[1].z}));
        Put(row,"transverse_axis",Vector(row,r.transverse_axis));Number(row,"reference_length_m",r.length_m);
        Put(row,"source_card_lines",Ids(row,{source.record.cards[0].source_line,source.record.cards[1].source_line}));
        Value value;value.CopyFrom(row,d.GetAllocator());connections.PushBack(value,d.GetAllocator());
        for(unsigned local=0;local<2;++local) {
            const auto& m=model->endpoint_mass()[2*i+local];const auto total=b.coefficients(m.global_node);
            auto values=Ids(d,{m.source_element_id,m.source_property_id,local,m.global_node,m.source_node_id});
            for(double x:{m.mass_kg,m.isotropic_inertia_kg_m2,total.mass,total.isotropic_inertia})values.PushBack(x,d.GetAllocator());
            endpoints.PushBack(values,d.GetAllocator());
        }
    }
    Put(d,"properties",std::move(properties));Put(d,"connections",std::move(connections));
    String(d,"endpoint_columns","source_element_id,generated_property_id,local_endpoint,global_node,source_node_id,mass_kg,inertia_kg_m2,total_node_mass_kg,total_node_inertia_kg_m2");
    Put(d,"endpoint_contributions",std::move(endpoints));return d;
}
} // namespace crash::output::assembly::wall_fields
