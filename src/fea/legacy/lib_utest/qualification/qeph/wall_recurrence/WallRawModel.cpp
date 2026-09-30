#include "WallRawJson.h"

namespace tl::qualification::qeph::wall_recurrence::raw_detail {
namespace {
template<class V> void Positions(io::Document& d,const char* name,const V* positions,unsigned count) {
  io::Value array(rapidjson::kArrayType);
  for(unsigned i=0;i<count;++i) {
    const double xyz[]{positions[i].x,positions[i].y,positions[i].z};
    array.PushBack(io::FiniteArray(d,xyz,3),d.GetAllocator());
  }
  d.AddMember(io::Value(name,d.GetAllocator()),array,d.GetAllocator());
}
io::Document NativeReference(const ReferenceData& reference,const std::array<unsigned,4>& nodes) {
  io::Document d; d.SetObject(); const auto& input=reference.input;
  Indices(d,"global_nodes",nodes.data(),4); Indices(d,"source_node_ids",input.node_ids.data(),4);
  Positions(d,"reference_positions_m",input.position.data(),4);
  io::Number(d,"density_kg_m3",input.density); io::Number(d,"young_modulus_pa",input.young_modulus);
  io::Number(d,"poisson_ratio",input.poisson_ratio); io::Number(d,"thickness_m",input.thickness);
  io::FiniteArray(d,"frame_row_major",reference.frame.v,9); io::Number(d,"area_m2",reference.area);
  Positions(d,"local_positions_m",reference.local_position.data(),4);
  io::FiniteArray(d,"derivative_x_m",reference.derivative_x.data(),4);
  io::FiniteArray(d,"derivative_y_m",reference.derivative_y.data(),4);
  io::FiniteArray(d,"nodal_mass_kg",reference.nodal_mass.data(),4);
  io::FiniteArray(d,"physical_inertia_kg_m2",reference.physical_inertia.data(),4);
  io::FiniteArray(d,"added_inertia_kg_m2",reference.added_inertia.data(),4);
  io::FiniteArray(d,"total_isotropic_inertia_kg_m2",reference.isotropic_inertia.data(),4); return d;
}
io::Document ContactReference(const contact::Q4ParametricParent& parent) {
  const auto& q=parent.intrinsic; const auto& identity=q.parent();
  io::Document d; d.SetObject(); io::Integer(d,"parent_element_id",identity.parent_element_id);
  io::Integer(d,"feature_id",identity.feature_id); io::Integer(d,"parent_face_id",identity.parent_face_id);
  Indices(d,"nodes",identity.nodes,4); io::Number(d,"half_thickness_m",identity.half_thickness);
  Field(d,"center_area_m2",Certificate(parent.area)); Field(d,"intrinsic_area_m2",Interval(q.area_enclosure()));
  Vec(d,"chart_direction",q.direction()); io::Number(d,"direction_norm_upper",q.direction_norm_upper());
  io::Value corners(rapidjson::kArrayType);
  for(unsigned i=0;i<4;++i) {
    io::Document c; c.SetObject(); Vec(c,"position_m",q.position(i)); Vec(c,"nominal_cross",q.nominal_corner(i));
    io::Value bounds(rapidjson::kArrayType);
    for(unsigned a=0;a<3;++a) Append(c,bounds,Interval(q.corner(i).component[a]));
    c.AddMember("cross_component_bounds",bounds,c.GetAllocator()); Append(d,corners,c);
  }
  d.AddMember("intrinsic_corners",corners,d.GetAllocator()); return d;
}
io::Document Dictionary(const recurrence::Coordinate& c) {
  const char* names[]{"position","orientation_tangent","velocity","spin","history","force_cache"};
  io::Require(static_cast<unsigned>(c.group)<6&&c.scale>0,"Invalid raw model dictionary");
  io::Document d; d.SetObject(); io::String(d,"group",names[static_cast<unsigned>(c.group)]);
  io::String(d,"name",c.name); io::String(d,"unit",c.unit); io::Number(d,"scale",c.scale);
  io::Integer(d,"entity",c.entity); io::Integer(d,"component",c.component);
  io::Boolean(d,"legacy_bq4_feedback_classification",c.feedback); return d;
}
}
io::Document DescribeModel(const WallRecurrenceModel& model) {
  io::Require(model.prepared(),"Missing immutable raw model");
  const auto& m=model.native(); io::Document d; d.SetObject();
  io::String(d,"model","cw1-coherent-broadside-8mps-penalty-target-v1");
  io::String(d,"contact_model",contact::NodalWallContactModel);
  io::String(d,"inertia_policy","native physical plus native area-added isotropic inertia");
  io::Number(d,"native_dm",.015); io::Number(d,"native_dn",.015);
  io::Integer(d,"cells",m.elements); io::Integer(d,"nodes",m.nodes); io::Integer(d,"dimension",m.dictionary.size());
  io::Boolean(d,"any_coordinate_reduced",false); io::Number(d,"impact_speed_m_s",ImpactSpeed);
  io::Number(d,"target_depth_m",TargetDepth); io::Number(d,"maximum_depth_m",model.law().maximum_penetration);
  io::Number(d,"initial_gap_m",InitialGap); io::Number(d,"wall_x_m",model.law().wall_x);
  io::Number(d,"kappa_n_m3",model.law().stiffness_per_area);
  io::Integer(d,"kappa_binary64_bits",io::Bits(model.law().stiffness_per_area));
  io::Number(d,"parent_force_error_n",model.law().parent_force_error);
  io::Number(d,"parent_energy_error_j",model.law().parent_energy_error);
  io::Number(d,"mass_rate_relative_spread_upper",model.rate_spread_upper());
  io::Number(d,"maximum_contact_frequency_s_inverse",model.maximum_frequency());
  io::Value chain(rapidjson::kArrayType),dictionary(rapidjson::kArrayType),native(rapidjson::kArrayType),contact_references(rapidjson::kArrayType);
  for(const auto x:model.penalty_chain()) Append(d,chain,Interval(x));
  for(const auto& c:m.dictionary) Append(d,dictionary,Dictionary(c));
  for(unsigned e=0;e<m.elements;++e) {
    Append(d,native,NativeReference(m.reference[e].data(),m.connectivity[e]));
    auto p=ContactReference(model.reference().parent(e));
    Field(p,"nodal_share_area_m2",Certificate(model.weights().parent(e).share)); Append(d,contact_references,p);
  }
  d.AddMember("penalty_interval_chain",chain,d.GetAllocator()); d.AddMember("dictionary",dictionary,d.GetAllocator());
  d.AddMember("native_references",native,d.GetAllocator()); d.AddMember("contact_references",contact_references,d.GetAllocator());
  Positions(d,"current_touching_baseline_m",m.position.data(),m.nodes);
  io::FiniteArray(d,"assembled_native_mass_kg",m.mass.data(),m.nodes);
  io::FiniteArray(d,"assembled_native_inertia_kg_m2",m.inertia.data(),m.nodes);
  Field(d,"total_contact_area_m2",Certificate(model.weights().total_area()));
  io::Value weights(rapidjson::kArrayType);
  for(unsigned n=0;n<m.nodes;++n) {
    auto node=DescribePoint(model.touching().nodes[n]);
    Field(node,"assembled_contact_area_m2",Certificate(model.weights().node(n).area));
    Field(node,"stiffness_over_native_mass_s_inverse_squared",Certificate(model.mass_rates()[n])); Append(d,weights,node);
  }
  d.AddMember("touching_node_coefficients",weights,d.GetAllocator());
  io::Value wall(rapidjson::kArrayType);
  for(const auto& face:model.wall().faces()) {
    io::Document f; f.SetObject(); io::Integer(f,"triangle_id",face.geometry.face_id);
    io::Integer(f,"source_quad_id",face.source_quad_id); io::Integer(f,"assembled_source_quad_id",face.assembled_source_quad_id);
    Indices(f,"geometry_vertex_ids",face.geometry.vertex_ids,3); Positions(f,"positions_m",face.geometry.vertices,3); Append(d,wall,f);
  }
  d.AddMember("synthetic_finite_wall_faces",wall,d.GetAllocator()); io::Number(d,"wall_tolerance_m",model.wall().tolerance());
  const auto& coverage=model.coverage();
  io::Require(coverage.mode==contact::PlanarWallBoxMode::Exact||
    coverage.mode==contact::PlanarWallBoxMode::ConservativeExpansion,"Invalid retained coverage mode");
  Vec(d,"covered_box_minimum_m",coverage.physical.minimum); Vec(d,"covered_box_maximum_m",coverage.physical.maximum);
  Vec(d,"query_box_minimum_m",coverage.query.minimum); Vec(d,"query_box_maximum_m",coverage.query.maximum);
  Vec(d,"query_lower_expansion_upper_m",coverage.lower_expansion_upper);
  Vec(d,"query_upper_expansion_upper_m",coverage.upper_expansion_upper);
  io::String(d,"coverage_mode",coverage.mode==contact::PlanarWallBoxMode::Exact?"exact":"conservative-expansion");
  io::Boolean(d,"motion_box_covered",coverage.covered); io::Number(d,"wall_clearance_m",WallClearance); return d;
}
} // namespace tl::qualification::qeph::wall_recurrence::raw_detail
