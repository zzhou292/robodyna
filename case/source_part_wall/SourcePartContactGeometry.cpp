#include "SourcePartContactGeometry.h"
#include "collision/Q4ParametricContact.h"
#include "collision/SurfaceMaterialMeasure.h"
#include <algorithm>

namespace crash::cases::source_part_wall {
struct SourcePartContactGeometry::Data {
    std::array<contact::Q4ParametricReference,source::ParentCount> quad;
    std::array<contact::T3MaterialMeasure,source::ParentCount> triangle;
    std::array<contact::NodalWallParentInput,source::ParentCount> inputs{};
    contact::NodalWallWeights weights;
    std::array<contact::Vec3,2> bounds;
    std::array<unsigned,source::ParentCount> source_to_weight{},weight_to_source{};
};
SourcePartContactGeometry::SourcePartContactGeometry()=default;
SourcePartContactGeometry::~SourcePartContactGeometry()=default;
bool SourcePartContactGeometry::prepared() const noexcept { return data_!=nullptr; }
const contact::NodalWallWeights* SourcePartContactGeometry::weights() const noexcept {
    return data_?&data_->weights:nullptr;
}
std::array<contact::Vec3,2> SourcePartContactGeometry::reference_bounds() const noexcept {
    return data_?data_->bounds:std::array<contact::Vec3,2>{};
}
bool SourcePartContactGeometry::ParentWeights(unsigned source_parent,contact::NodalWallWeights* output) const {
    return data_&&output&&source_parent<source::ParentCount&&
        output->Initialize(source::NodeCount,&data_->inputs[source_parent],1).status==contact::NodalWallStatus::Ok;
}
unsigned SourcePartContactGeometry::weight_index(unsigned parent) const noexcept {
    return data_&&parent<source::ParentCount?data_->source_to_weight[parent]:UINT32_MAX;
}
unsigned SourcePartContactGeometry::source_parent_index(unsigned index) const noexcept {
    return data_&&index<source::ParentCount?data_->weight_to_source[index]:UINT32_MAX;
}
bool SourcePartContactGeometry::Initialize(const source::SourcePartContactFixture& input,std::string& diagnostic) {
    diagnostic.clear();
    if(data_||!input.prepared()) {
        diagnostic="Source preparation requires a fresh object and authenticated input";return false;
    }
    auto next=std::make_unique<Data>();
    // Original per-parent operations and argument order are shared verbatim
    // with the former qualification preparation. No reordering/area refit.
    for(unsigned p=0;p<source::ParentCount;++p) {
        if(input.parents()[p].arity==4) {
            contact::SurfaceQ4 parent;
            if(!input.q4_parent(p,parent)||
               next->quad[p].Initialize(input.positions(),&parent,1).status!=contact::Q4ParametricStatus::Ok) {
                diagnostic="Original Q4 immutable area failed";return false;
            }
            next->inputs[p]={&next->quad[p],0,nullptr};
        } else {
            contact::SurfaceTriangle parent;
            if(!input.t3_parent(p,parent)||
               contact::PrepareT3MaterialMeasure(input.positions(),parent,&next->triangle[p])!=contact::SurfaceMeasureStatus::Ok) {
                diagnostic="Original native T3 immutable area failed";return false;
            }
            next->inputs[p]={nullptr,0,&next->triangle[p]};
        }
    }
    if(next->weights.Initialize(source::NodeCount,next->inputs.data(),source::ParentCount).status!=contact::NodalWallStatus::Ok||
       next->weights.node_count()!=source::NodeCount||next->weights.parent_count()!=source::ParentCount) {
        diagnostic="Source nodal contact weights failed or omitted a physical node";return false;
    }
    for(unsigned n=0;n<source::NodeCount;++n)if(next->weights.node(n).node!=n) {
        diagnostic="Source dense node order changed";return false;
    }
    for(unsigned w=0;w<source::ParentCount;++w) {
        unsigned original=0;
        while(original<source::ParentCount&&input.parents()[original].source_id!=next->weights.parent(w).parent_element_id)++original;
        if(original==source::ParentCount) {diagnostic="Contact weights changed source parent identity";return false;}
        next->weight_to_source[w]=original;next->source_to_weight[original]=w;
    }
    const auto& x=input.coordinates();next->bounds={contact::Vec3{x[0],x[1],x[2]},contact::Vec3{x[0],x[1],x[2]}};
    for(unsigned n=1;n<source::NodeCount;++n) {
        next->bounds[0].x=std::min(next->bounds[0].x,x[3*n]);next->bounds[1].x=std::max(next->bounds[1].x,x[3*n]);
        next->bounds[0].y=std::min(next->bounds[0].y,x[3*n+1]);next->bounds[1].y=std::max(next->bounds[1].y,x[3*n+1]);
        next->bounds[0].z=std::min(next->bounds[0].z,x[3*n+2]);next->bounds[1].z=std::max(next->bounds[1].z,x[3*n+2]);
    }
    data_=std::move(next);return true;
}
contact::PlanarContactReport SourcePartContactGeometry::CheckWallCoverage(const contact::PlanarWallGeometry& wall,
    contact::Vec3 minimum,contact::Vec3 maximum,double clearance,std::uint64_t feature_id,contact::PlanarWallBoxCoverage* output) const {
    using Status=contact::PlanarContactStatus;
    if(!data_||!wall.initialized())return {Status::NotInitialized,"Source or finite wall is not prepared"};
    if(!output||!feature_id||!contact::IsFinite(minimum)||!contact::IsFinite(maximum)||
       minimum.x>data_->bounds[0].x||minimum.y>data_->bounds[0].y||minimum.z>data_->bounds[0].z||
       maximum.x<data_->bounds[1].x||maximum.y<data_->bounds[1].y||maximum.z<data_->bounds[1].z)
        return {Status::InvalidInput,"Declared finite motion box does not contain the entire original part"};
    const contact::PlanarWallBox projected{{wall.wall_x(),minimum.y,minimum.z},{wall.wall_x(),maximum.y,maximum.z}};
    return contact::CheckPlanarWallBox(wall,projected,clearance,feature_id,
        contact::PlanarWallBoxMode::ConservativeExpansion,output);
}
} // namespace crash::cases::source_part_wall
