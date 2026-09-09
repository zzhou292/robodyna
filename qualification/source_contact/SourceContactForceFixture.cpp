#include "SourceContactForceFixture.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace crash::qualification::source_contact::force {
namespace {
sc::Vec3 Cross(sc::Vec3 a,sc::Vec3 b) {return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
}
bool FixtureMass::Initialize(const sf::SourcePartContactFixture& source) {
        FixtureMass next;
        for (unsigned p=0;p<sf::ParentCount;++p) {
            const auto& parent=source.parents()[p];
            const double share=source.surface_mass().parent_mass_kg[p]/parent.arity;
            if (!std::isfinite(share)||share<=0) return false;
            for (unsigned n=0;n<parent.arity;++n) next.mass[parent.local_node_indices[n]]+=share;
        }
        for (unsigned n=0;n<sf::NodeCount;++n) {
            if (!std::isfinite(next.mass[n])||next.mass[n]<=0) return false;
            next.inverse[n]=1/next.mass[n];
            if (!std::isfinite(next.inverse[n])||next.inverse[n]<=0) return false;
        }
        *this=next;return true;
    }
bool Harness::EvaluateAll(const Coordinates& base,const Coordinates& endpoint,const Coordinates& velocity,
                     const sc::PlanarWallGeometry& wall,Aggregate* output) {
        Aggregate next;
        for (unsigned p=0;p<sf::ParentCount;++p) {
            failed_parent=p;
            if (!EvaluateParent(p,base,endpoint,velocity,wall,&next.parents[p])) return false;
            const auto& parent=next.parents[p];next.wall_reaction.x+=parent.resultant.value;
            for (unsigned n=0;n<parent.arity;++n) {
                const auto node=parent.nodes[n];const auto force=parent.force[n];
                next.forces[node]=sc::Add(next.forces[node],force);
                next.wall_moment=sc::Subtract(next.wall_moment,Cross(View(endpoint).at(node),force));
                next.power+=sc::Dot(View(velocity).at(node),force);
            }
        }
        next.valid=true;*output=next;failed_parent=UINT32_MAX;return true;
    }
// One original parent at a time through the owning finite-wall dispatcher.
// No triangle preflight/coverage or force integration is copied into this test.
bool Harness::EvaluateParent(unsigned p,const Coordinates& base,const Coordinates& endpoint,
                             const Coordinates& velocity,const sc::PlanarWallGeometry& wall,ParentForce* output) {
    diagnostic.clear();
    if (!output || p>=sf::ParentCount) return false;
    const auto& binding=source.parents()[p];
    sc::Q4ParametricReference quad_reference;sc::T3MaterialMeasure triangle_reference;
    sc::SurfaceQ4 quad;sc::SurfaceTriangle triangle;sc::PrescribedSurfaceParent request;
    ParentForce next;next.arity=binding.arity;
    if (binding.arity==4) {
        if (!source.q4_parent(p,quad)) return false;
        const auto prepared=quad_reference.Initialize(source.positions(),&quad,1);
        if (prepared.status!=sc::Q4ParametricStatus::Ok) {diagnostic=prepared.message;return false;}
        next.area=quad_reference.parent(0).area;
        request.family=sc::PrescribedSurfaceFamily::Q4CenterAreaUniformNatural;
        request.q4={&quad_reference,0,&quad,&quad};
    } else {
        if (!source.t3_parent(p,triangle)) return false;
        if (sc::PrepareT3MaterialMeasure(source.positions(),triangle,&triangle_reference)!=sc::SurfaceMeasureStatus::Ok ||
            !sc::q4_bounds::Certify(.5*triangle_reference.density().value,triangle_reference.area_enclosure(),&next.area)) {
            diagnostic="Source native triangle reference preparation failed";return false;
        }
        request.family=sc::PrescribedSurfaceFamily::T3NativeLinear;
        request.t3={&triangle_reference,&triangle,&triangle};
    }
    sc::PrescribedSurfaceConfig config;
    config.stiffness_per_area=Stiffness;config.maximum_penetration=Cap;config.exposed_clearance=Clearance;
    config.q4.force_error=ForceBudget;config.q4.energy_error=EnergyBudget;
    config.t3={ForceBudget,EnergyBudget};
    const sc::PrescribedSurfaceInput input{View(base),View(velocity),View(endpoint),View(velocity),mass.view(),&request,1};
    sc::PrescribedSurfaceResult result;
    const auto report=sc::IntegratePrescribedSurfaceContact(wall,input,config,31,scratch->view(),&result);
    if (report.status!=sc::PrescribedSurfaceStatus::Ok) {
        std::ostringstream message;message<<report.message<<"; source="<<binding.source_id
            <<" status="<<static_cast<int>(report.status)<<" q4="<<static_cast<int>(report.q4.status)
            <<" t3="<<static_cast<int>(report.t3.status);diagnostic=message.str();return false;
    }
    if (!result.valid || result.parent_count!=1 || result.parents[0].family!=request.family ||
        !result.parents[0].coverage.covered) {diagnostic="Incomplete owning dispatcher result";return false;}
    if (binding.arity==4) {
        const auto& rectangular=result.parents[0].q4;const auto& integral=rectangular.integration;
        for (unsigned n=0;n<4;++n) {
            next.nodes[n]=integral.nodal.nodes[n];next.force[n]=integral.nodal.forces[n];next.magnitude[n]=integral.force[n];
            const auto c=integral.nodal.couples[n];if (c.x!=0||c.y!=0||c.z!=0) return false;
        }
        next.source_id=integral.parent_element_id;next.feature_id=integral.feature_id;
        next.base_epoch=integral.base_epoch;next.attempt=integral.attempt;
        next.resultant=integral.resultant;next.potential=integral.potential;next.active_area=integral.active_area;
        next.cells=integral.leaf_count;next.visits=integral.visited;
        next.depth_u=rectangular.deepest_u;next.depth_v=rectangular.deepest_v;next.valid=integral.valid;
    } else {
        const auto& integral=result.parents[0].t3;
        for (unsigned n=0;n<3;++n) {
            next.nodes[n]=integral.nodal.nodes[n];next.force[n]=integral.nodal.forces[n];next.magnitude[n]=integral.force[n];
        }
        if (integral.parent_face_id!=triangle.parent_face_id) return false;
        next.source_id=integral.parent_element_id;next.feature_id=integral.feature_id;
        next.base_epoch=integral.base_epoch;next.attempt=integral.attempt;
        next.resultant=integral.resultant;next.potential=integral.potential;next.active_area=integral.active_area;
        next.cells=integral.subtriangle_count;next.visits=integral.sample_count;next.valid=integral.valid;
    }
    if (!next.valid) {diagnostic="Invalid native integration result";return false;}
    *output=next;return true;
}

Coordinates Shift(const sf::SourcePartContactFixture& source,double translation) {
    auto x=source.coordinates();
    for (unsigned n=0;n<sf::NodeCount;++n) x[3*n]+=translation;
    return x;
}
Coordinates Velocity(double translation) {
    Coordinates v{};
    // Explicit straight prescribed path over1second, not a solver clock/step.
    for (unsigned n=0;n<sf::NodeCount;++n) v[3*n]=translation;
    return v;
}
double ParentShift(const sf::SourcePartContactFixture& source,unsigned p,double wall_x) {
    double maximum=-std::numeric_limits<double>::infinity();const auto& parent=source.parents()[p];
    for (unsigned n=0;n<parent.arity;++n) maximum=std::max(maximum,source.positions().at(parent.local_node_indices[n]).x);
    return (wall_x-maximum)+Depth;
}
double WholeShift(const sf::SourcePartContactFixture& source,double wall_x) {
    double maximum=-std::numeric_limits<double>::infinity();
    for (unsigned n=0;n<sf::NodeCount;++n) maximum=std::max(maximum,source.positions().at(n).x);
    return (wall_x-maximum)+Depth;
}
} // namespace crash::qualification::source_contact::force
