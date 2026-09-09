#pragma once
#include "chrono/core/ChMatrix.h"
#include "GuidedPlateStudy.h"
#include "chrono/core/ChQuaternion.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace crash::case_data::study_test {
namespace ct=tlfea::contact;
namespace sh=tl::fea::reissner;
namespace ref=crash::reference;
// Synthetic observer inputs, deliberately independent of GPU/solver execution.
// These tests verify accounting/association, not the supplied fields' mechanics.
inline GuidedStudyCertificate Cert(double value,double lower,double upper) {
    const long double distance=std::max(std::abs(static_cast<long double>(value)-lower),
                                       std::abs(static_cast<long double>(upper)-value));
    double error=static_cast<double>(distance);
    if(error)error=std::nextafter(error,std::numeric_limits<double>::infinity());
    return {value,lower,upper,error};
}
inline GuidedStudyConfig Config(unsigned refinement=1,std::uint64_t owner=7) {
    GuidedStudyConfig c;c.owner_id=owner;c.qualification_id=kGuidedPlateQualification;c.wall_binding_id=23;c.base_steps=200;c.refinement=refinement;
    c.fixed_dt=.001/refinement;c.horizon=.2;c.initial_energy=1;c.wall_x=.05;c.experiment_sha256=std::string(64,'a');
    const unsigned nodes[2][4]={{0,1,2,3},{4,0,3,5}};
    for(unsigned n=0;n<6;++n) {c.reference_position[3*n]=.04;c.reference_rotation[4*n]=1;}
    for(unsigned e=0;e<2;++e) {
        auto& p=c.contact_reference[e];p.parent.feature_id=1001+e;p.parent.parent_element_id=101+e;p.covered=true;
        p.projected_area=.01;p.area_enclosure={.00999999999999999,.01000000000000001};
        for(unsigned j=0;j<4;++j) {p.parent.nodes[j]=nodes[e][j];p.reference_projection[j]={c.wall_x,0,0};}
    }
    // Independent exact arithmetic for these two equal bounds, then conservative
    // binary64 endpoints match the donor addition (doubling is exact).
    c.total_reference_area={2*c.contact_reference[0].area_enclosure.lower,2*c.contact_reference[0].area_enclosure.upper};return c;
}
inline GuidedPlateFrame Frame(const GuidedStudyConfig& c,std::uint64_t epoch,const GuidedPlateFrame* before=nullptr) {
    GuidedPlateFrame f;auto& m=f.metrics;auto& t=m.stamp;
    t.owner_id=c.owner_id;t.epoch=epoch;t.node_count=6;t.has_rotations=true;t.fixed_dt=c.fixed_dt;
    t.time=before?before->stamp.time+c.fixed_dt:0;t.reactions_valid=epoch!=0;
    t.reaction_base_epoch=epoch?epoch-1:0;t.reaction_time=before?before->stamp.time:0;
    m.required_steps=c.base_steps*c.refinement;m.initial_energy=1;
    auto& s=m.shell;s.valid=true;s.owner_id=c.owner_id;s.configuration_id=c.qualification_id;s.attempt=epoch+1;
    s.base_epoch=epoch?epoch-1:0;s.phase=epoch?sh::ShellBatchPhase::kPreparedCandidate:sh::ShellBatchPhase::kAcceptedBase;
    auto& d=m.contact;d.valid=true;d.owner_id=c.owner_id;d.configuration_id=c.qualification_id;d.wall_binding_id=c.wall_binding_id;
    d.integration_backend=c.integration_backend;d.leaves=2;d.visited=2;
    d.attempt=s.attempt;d.base_epoch=s.base_epoch;d.phase=epoch?ct::Q4PlanarContactPhase::PreparedCandidate:ct::Q4PlanarContactPhase::AcceptedBase;
    d.parent_count=d.covered_count=2;
    const bool active=epoch>50*c.refinement&&epoch<=110*c.refinement;
    d.wall_reaction.x=active?1:0;d.force_on_surface.x=-d.wall_reaction.x;d.force_error.x=active?1e-5:0;
    d.potential=active?Cert(.001,.001-1e-8,.001+1e-8):Cert(0,0,0);
    d.active_area=active?GuidedStudyInterval{.004,.006}:GuidedStudyInterval{};d.maximum_penetration=active?1e-5:0;
    s.elastic_energy=1-d.potential.value;s.bending_energy=.01;m.work.total_energy=s.elastic_energy+d.potential.value;
    if(before) {
        m.applied_contact=before->metrics.contact;m.applied_contact.attempt=s.attempt;m.applied_contact.base_epoch=epoch-1;
        m.applied_contact.phase=ct::Q4PlanarContactPhase::AcceptedBase;
        m.wall_impulse.x=before->metrics.wall_impulse.x+c.fixed_dt*m.applied_contact.wall_reaction.x;
    }
    f.stamp=t;f.element_association=s;f.contact_association=d;
    const double phase=double(epoch)/(c.base_steps*c.refinement),pi=std::acos(-1.);
    const double displacement=-.002*std::cos(2*pi*phase),velocity=.002*2*pi/.2*std::sin(2*pi*phase);
    for(unsigned n=0;n<6;++n) {
        const double scale=n==1||n==2?0:n==0||n==3?.35:1;
        for(unsigned j=0;j<3;++j)f.position[3*n+j]=c.reference_position[3*n+j];
        f.position[3*n]+=scale*displacement;f.velocity[3*n]=scale*velocity;
        const double angle=10*scale*displacement;
        const chrono::ChQuaterniond turn(std::cos(.5*angle),0,0,std::sin(.5*angle));
        const chrono::ChQuaterniond reference(c.reference_rotation[4*n],c.reference_rotation[4*n+1],c.reference_rotation[4*n+2],c.reference_rotation[4*n+3]);
        const auto q=turn*reference;for(unsigned j=0;j<4;++j)f.rotation[4*n+j]=q[j];
    }
    for(unsigned e=0;e<2;++e) {
        auto& p=f.parent[e];p.covered=true;p.integration_backend=c.integration_backend;
        auto& r=p.integration;r.valid=true;r.leaf_count=1;r.visited=1;
        r.feature_id=c.contact_reference[e].parent.feature_id;r.parent_element_id=c.contact_reference[e].parent.parent_element_id;
        r.base_epoch=s.base_epoch;r.attempt=s.attempt;r.active_area=active?GuidedStudyInterval{.002,.003}:GuidedStudyInterval{};
        for(unsigned n=0;n<4;++n) {const double force=active?.05*(n+1):0;r.force[n]=Cert(force,force,force);}
    }
    return f;
}
inline void SetContact(GuidedPlateFrame& f,double force,double error,GuidedStudyCertificate potential) {
    auto& m=f.metrics;m.contact.wall_reaction.x=force;m.contact.force_error.x=error;m.contact.potential=potential;
    m.shell.elastic_energy=1-potential.value;m.work.total_energy=m.shell.elastic_energy+potential.value;
    f.contact_association=m.contact;f.element_association=m.shell;
}
inline GuidedStudyData Run(const GuidedStudyConfig& c) {
    auto frame=Frame(c,0);GuidedPlateStudy recorder;std::string error;
    if(!recorder.Initialize(c,frame.metrics,frame,error))throw std::runtime_error(error);
    for(std::uint64_t e=1;e<=c.base_steps*c.refinement;++e) {
        auto next=Frame(c,e,&frame);
        if(!recorder.Record(next.metrics,recorder.NeedsSample(e)?&next:nullptr,error))throw std::runtime_error(error);
        frame=next;
    }
    GuidedStudyData result;if(!recorder.Finish(result,error))throw std::runtime_error(error);return result;
}

inline GuidedStudyData Run(unsigned refinement,std::uint64_t owner) { return Run(Config(refinement,owner)); }
} // namespace crash::case_data::study_test
