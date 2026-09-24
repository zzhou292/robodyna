// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../Fixture.h"
#include <cstring>
#include <limits>
#include <locale>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace exact_storage_discovery {
namespace ct=tlfea::contact;
inline void Require(bool value,const char* message) {
    if(!value)throw std::runtime_error(message);
}
inline std::uint64_t Bits(double value) noexcept {
    static_assert(sizeof(double)==sizeof(std::uint64_t) && std::numeric_limits<double>::is_iec559);
    std::uint64_t result=0;std::memcpy(&result,&value,sizeof(result));return result;
}
inline void Text(std::ostream& out,const char* text) {
    if(!text){out<<"null";return;}
    constexpr char hex[]="0123456789abcdef";
    out<<'"';
    for(std::size_t i=0;i<4096;++i) {
        const auto ch=static_cast<unsigned char>(text[i]);
        if(!ch){out<<'"';return;}
        if(ch=='"' || ch=='\\')out<<'\\'<<char(ch);
        else if(ch<0x20 || ch>=0x7f)out<<"\\u00"<<hex[ch>>4]<<hex[ch&15];
        else out<<char(ch);
    }
    throw std::runtime_error("Discovery parity message exceeds fixed bound");
}
inline void Vertex(std::ostream& out,const ct::FacetVertexKey& value) {
    out<<"{\"source_instance_id\":"<<value.source_instance_id<<",\"kind\":"<<unsigned(value.kind)
       <<",\"first\":"<<value.first<<",\"second\":"<<value.second
       <<",\"numerator\":"<<value.numerator<<",\"denominator\":"<<value.denominator
       <<",\"level\":"<<value.level<<",\"grid_i\":"<<value.grid_i<<",\"grid_j\":"<<value.grid_j<<'}';
}
inline void Edge(std::ostream& out,const ct::FacetEdgeKey& value) {
    out<<"{\"parent_eid\":"<<value.parent_eid<<",\"parent_boundary\":"<<(value.parent_boundary?"true":"false")<<",\"endpoints\":[";
    Vertex(out,value.endpoints[0]);out<<',';Vertex(out,value.endpoints[1]);out<<"]}";
}
inline void Triangle(std::ostream& out,const ct::FixedTriangleKey& value) {
    out<<"{\"source_instance_id\":"<<value.source_instance_id<<",\"parent_eid\":"<<value.parent_eid
       <<",\"level\":"<<value.level<<",\"local_facet\":"<<value.local_facet<<'}';
}
inline void Stratum(std::ostream& out,const ct::FixedTriangleStratumKey& value) {
    out<<"{\"kind\":"<<unsigned(value.kind)<<",\"active\":";
    switch(value.kind) {
      case ct::FixedTriangleStratumKind::Vertex:Vertex(out,value.vertex);break;
      case ct::FixedTriangleStratumKind::Edge:Edge(out,value.edge);break;
      case ct::FixedTriangleStratumKind::Face:Triangle(out,value.face);break;
      default:throw std::runtime_error("Invalid published stratum kind");
    }
    out<<'}';
}
inline void FeatureKey(std::ostream& out,const ct::FixedTriangleFeatureKey& value) {
    out<<"{\"kind\":"<<unsigned(value.kind)<<",\"active\":";
    switch(value.kind) {
      case ct::FixedTriangleCandidateKind::VertexFace:
        out<<"{\"vertex\":";Vertex(out,value.vertex_face.vertex);
        out<<",\"target\":";Stratum(out,value.vertex_face.target);out<<'}';break;
      case ct::FixedTriangleCandidateKind::EdgeEdge:
        out<<'[';Edge(out,value.edge_edge.edges[0]);out<<',';Edge(out,value.edge_edge.edges[1]);out<<']';break;
      default:throw std::runtime_error("Invalid published candidate kind");
    }
    out<<'}';
}
inline void Point(std::ostream& out,ct::Vec3 value) {
    out<<'['<<Bits(value.x)<<','<<Bits(value.y)<<','<<Bits(value.z)<<']';
}
inline void Feature(std::ostream& out,const ct::FixedTriangleFeatureCandidate& value) {
    out<<"{\"key\":";FeatureKey(out,value.key);out<<",\"triangles\":[";
    Triangle(out,value.triangles[0]);out<<',';Triangle(out,value.triangles[1]);
    out<<"],\"local_features\":["<<value.local_features[0]<<','<<value.local_features[1]
       <<"],\"point_bits\":[";Point(out,value.points[0]);out<<',';Point(out,value.points[1]);
    out<<"],\"face_weight_bits\":["<<Bits(value.face_weights[0])<<','<<Bits(value.face_weights[1])<<','<<Bits(value.face_weights[2])
       <<"],\"edge_parameter_bits\":["<<Bits(value.edge_parameters[0])<<','<<Bits(value.edge_parameters[1])
       <<"],\"distance_bits\":"<<Bits(value.distance_m)<<",\"representation_error_bits\":"<<Bits(value.representation_error_m)<<'}';
}
inline void Intersection(std::ostream& out,const ct::FixedTriangleIntersection& value) {
    out<<"{\"triangles\":[";Triangle(out,value.triangles[0]);out<<',';Triangle(out,value.triangles[1]);
    out<<"],\"kind\":"<<unsigned(value.kind)<<",\"local_exclusion\":"<<unsigned(value.local_exclusion)<<'}';
}
inline void Report(std::ostream& out,const ct::FixedTriangleDiscoveryReport& value) {
    out<<"{\"status\":"<<unsigned(value.status)<<",\"input_pair\":"<<value.input_pair
       <<",\"input_task\":"<<value.input_task<<",\"arithmetic_reason\":"<<unsigned(value.arithmetic_reason)
       <<",\"feature_tasks\":"<<value.feature_tasks<<",\"triangle_references\":"<<value.triangle_references
       <<",\"triangles\":"<<value.triangles<<",\"vertex_references\":"<<value.vertex_references
       <<",\"vertices\":"<<value.vertices<<",\"edge_references\":"<<value.edge_references<<",\"edges\":"<<value.edges
       <<",\"raw_feature_candidates\":"<<value.raw_feature_candidates<<",\"feature_candidates\":"<<value.feature_candidates
       <<",\"raw_intersections\":"<<value.raw_intersections<<",\"intersections\":"<<value.intersections
       <<",\"message\":";Text(out,value.message);
    out<<",\"potential_tasks\":"<<value.potential_tasks<<",\"local_masked_tasks\":"<<value.local_masked_tasks
       <<",\"exact_executed_tasks\":"<<value.exact_executed_tasks<<'}';
}
inline std::string Publication(const ct::FixedTriangleFeatureDiscovery& owner) {
    const auto features=owner.features();const auto intersections=owner.intersections();
    Require(features.count<=128 && intersections.count<=16,"Parity publication exceeds fixed corpus bound");
    Require((!features.count || features.data) && (!intersections.count || intersections.data),"Missing published data");
    std::ostringstream out;out.imbue(std::locale::classic());
    out<<"{\"features\":{\"complete\":"<<(features.complete?"true":"false")<<",\"count\":"<<features.count
       <<",\"data_present\":"<<(features.data?"true":"false")<<",\"values\":[";
    for(std::size_t i=0;i<features.count;++i){if(i)out<<',';Feature(out,features.data[i]);}
    out<<"]},\"intersections\":{\"complete\":"<<(intersections.complete?"true":"false")<<",\"count\":"<<intersections.count
       <<",\"data_present\":"<<(intersections.data?"true":"false")<<",\"values\":[";
    for(std::size_t i=0;i<intersections.count;++i){if(i)out<<',';Intersection(out,intersections.data[i]);}
    out<<"]}";
    return out.str();
}
class Writer {
 public:
    explicit Writer(std::ostream& out):out_(out){out_.imbue(std::locale::classic());}
    void Begin(){Write("{\"schema\":\"fixed_triangle_discovery_parity.v1\",\"scope\":\"complete fieldwise reports and publications; binary64 bits; no timings\"}");}
    void Emit(const char* scenario,unsigned workers,unsigned permutation,const char* action,
              const ct::FixedTriangleDiscoveryReport& report,const ct::FixedTriangleFeatureDiscovery& owner) {
        std::ostringstream line;line.imbue(std::locale::classic());
        line<<"{\"scenario\":";Text(line,scenario);line<<",\"workers\":"<<workers<<",\"permutation\":"<<permutation<<",\"action\":";Text(line,action);
        line<<",\"report\":";Report(line,report);line<<",\"publication\":"<<Publication(owner)<<'}';
        Write(line.str());++records_;
    }
    void End(){Write("{\"complete\":true,\"records\":"+std::to_string(records_)+"}");}
 private:
    void Write(const std::string& value) {
        constexpr std::size_t cap=16u<<20;
        Require(value.size()<cap && bytes_<=cap-value.size()-1,"Parity output exceeds fixed16MiB cap");
        out_<<value<<'\n';Require(bool(out_),"Parity output write failed");bytes_+=value.size()+1;
    }
    std::ostream& out_;std::size_t bytes_=0,records_=0;
};
} // namespace exact_storage_discovery
