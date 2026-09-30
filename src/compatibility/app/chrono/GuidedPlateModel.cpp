#include "GuidedPlateModel.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace crash::reference {
namespace sc=tlfea::contact;
struct GuidedPlateModel::Impl {
    GuidedPlateData data;
    sc::PlanarWallGeometry wall;
    std::unique_ptr<ElasticCouponModel> shell;
    sc::Q4PlanarGeometry contact;
    sc::Q4PlanarStiffness stiffness;

    Impl(sc::PlanarWallView input,std::uint64_t binding,GuidedPlateExperiment experiment) {
        const auto* spec=FindGuidedPlateExperiment(experiment);
        if (!spec) throw std::invalid_argument("Unknown guided plate experiment");
        data.experiment=experiment; data.qualification_id=spec->qualification_id;
        data.stiffness_per_area=spec->stiffness_per_area; data.target_penetration=spec->target_penetration;
        if (!binding) throw std::invalid_argument("Guided plate requires a stable wall binding identity");
        const auto wall_report=wall.Initialize(input);
        if (wall_report.status!=sc::PlanarContactStatus::Ok)
            throw std::invalid_argument(std::string("Guided plate finite wall rejected: ")+wall_report.message);
        double low_y=std::numeric_limits<double>::infinity(),high_y=-low_y,low_z=low_y,high_z=-low_y;
        for (const auto& face:wall.faces()) for (const auto& point:face.geometry.vertices) {
            low_y=std::min(low_y,point.y); high_y=std::max(high_y,point.y);
            low_z=std::min(low_z,point.z); high_z=std::max(high_z,point.z);
        }
        const double center_y=.5*low_y+.5*high_y,center_z=.5*low_z+.5*high_z;
        data.pose={{.5,.5,.5,.5},{wall.wall_x()-data.initial_gap,center_y-.5*ElasticCouponData::length,center_z}};
        if (!(data.pose.translation.x<wall.wall_x()))
            throw std::invalid_argument("Guided plate initial gap is unresolved at the supplied wall coordinate");
        data.wall_binding_id=binding;
        shell=std::make_unique<ElasticCouponModel>(data.pose);
        const auto& reference=shell->data();
        std::array<double,3*kCouponNodes> x{},v{};
        for (std::size_t node=0;node<kCouponNodes;++node) {
            const auto position=reference.reference_configuration.position[node];
            x[3*node]=position.x; x[3*node+1]=position.y; x[3*node+2]=position.z;
        }
        for (std::size_t e=0;e<kCouponElements;++e) {
            auto& parent=data.parents[e];
            // Explicit synthetic model identities, independent of wall IDs.
            parent.feature_id=1001+e; parent.parent_element_id=101+e; parent.parent_face_id=0;
            for (std::size_t n=0;n<4;++n) parent.nodes[n]=static_cast<std::uint32_t>(reference.connectivity[e][n]);
        }
        const sc::Q4SurfaceView surface{{x.data(),kCouponNodes,3,1},{v.data(),kCouponNodes,3,1},data.parents.data(),kCouponElements};
        const sc::Q4FixedYZMassView mass{reference.inverse_mass.data(),data.translation_fixed_bits.data(),kCouponNodes,0};
        const auto report=contact.Initialize(wall,surface,mass,data.exposed_clearance);
        if (report.status!=sc::PlanarContactStatus::Ok)
            throw std::invalid_argument(std::string("Guided plate footprint rejected: ")+report.message);
        for (std::size_t e=0;e<kCouponElements;++e)
            if (!contact.view().parents[e].covered)
                throw std::invalid_argument("Derived guided plate placement is not wholly covered by the actual finite wall");
        if (sc::BuildQ4PlanarStiffness(contact.view(),mass,data.stiffness_per_area,&stiffness)!=sc::PlanarContactStatus::Ok ||
            !std::isfinite(stiffness.rate_bound) || stiffness.rate_bound<=0)
            throw std::invalid_argument("Guided plate contact stiffness/mass bound is invalid");
        data.integration.force_error=spec->force_error;
        data.integration.energy_error=spec->energy_error;
    }
};
GuidedPlateModel::GuidedPlateModel(sc::PlanarWallView wall,std::uint64_t binding,GuidedPlateExperiment experiment)
    : impl_(std::make_unique<Impl>(wall,binding,experiment)) {}
GuidedPlateModel::~GuidedPlateModel()=default;
const GuidedPlateData& GuidedPlateModel::data() const { return impl_->data; }
const ElasticCouponModel& GuidedPlateModel::shell() const { return *impl_->shell; }
const sc::PlanarWallGeometry& GuidedPlateModel::wall() const { return impl_->wall; }
const sc::Q4PlanarGeometry& GuidedPlateModel::contact_geometry() const { return impl_->contact; }
const sc::Q4PlanarStiffness& GuidedPlateModel::contact_stiffness() const { return impl_->stiffness; }

patch_audit::Layout<kGuidedPlateDofs> GuidedPlateLayout() {
    patch_audit::Layout<kGuidedPlateDofs> result;
    for (std::size_t free=0;free<kCouponFreeNodes.size();++free) {
        result[4*free]={kCouponFreeNodes[free],0};
        for (unsigned c=0;c<3;++c) result[4*free+c+1]={kCouponFreeNodes[free],c+3};
    }
    return result;
}
ElasticCouponStatus ApplyGuidedPlateIncrement(const ElasticCouponConfiguration& base,
                                             const std::array<double,kGuidedPlateDofs>& increment,double scale,
                                             ElasticCouponConfiguration& output,std::string& diagnostic) {
    const auto layout=GuidedPlateLayout();
    return patch_audit::ApplyIncrement(base,layout.data(),layout.size(),increment.data(),scale,output,diagnostic);
}
}  // namespace crash::reference
