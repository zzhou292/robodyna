#pragma once
#include "../Fields.h"
#include "../Buffers.h"
#include "output/full_shell/tests/TestSupport.h"

namespace crash::output::physical_frames::test {
namespace f=tl::fea;
using L=f::ShellSectionLaw;
struct Fixture {
    std::vector<records::ParentPoints> parents{
        {105,10,2,QephFamily,0,records::PlasticField::NotApplicable},
        {101,11,9,QbatFamily,4,records::PlasticField::NativeEquivalentPlasticStrain},
        {109,12,9,T3Family,1,records::PlasticField::NativeEquivalentPlasticStrain},
        {104,13,2,QephFamily,3,records::PlasticField::NotApplicable},
        {102,14,2,QephFamily,3,records::PlasticField::NativeEquivalentPlasticStrain}};
    std::vector<ParentField> mapping{{QephFamily,2,L::RigidSkin},{QbatFamily,0,L::Law44QbatFourInPlane},
        {T3Family,0,L::Law44Nip1},{QephFamily,0,L::LayeredLaw1Nip3},{QephFamily,1,L::LayeredLaw44Nip3}};
    records::Context context=records::Context::Create(records::test::Id(),3,parents.data(),parents.size(),.125);
    detail::FrameBuffers buffers{context};
    f::ShellBatchLayeredSection q[3],t[1];
    f::qbat::BatchResult b[1];
    std::uint8_t qa[3]{1,0,1},ta[1]{1},ba[1]{1};
    Fixture() {
        q[0]=f::ShellBatchLayeredSection::Elastic({});
        f::ShellBatchSectionState plastic;
        for(unsigned k=0;k<3;++k)plastic.history.point[k].plastic_strain=.1*(k+1);
        q[1]=f::ShellBatchLayeredSection::Plastic(plastic);
        q[2]=f::ShellBatchLayeredSection::RigidSkin();
        f::ShellBatchOnePointSectionState one;
        one.point.saved.plastic_strain=.875;
        t[0]=f::ShellBatchLayeredSection::OnePoint(one);
        for(unsigned k=0;k<4;++k)b[0].history.point[k].material.plastic_strain=.02*(k+1);
    }
    void Stage() {
        const double x[15]{9,8,7,-0.,1,2,10,11,12,3,4,5,6,7,8};
        detail::StagePositions({3,1,4},x,5,buffers.Staging());
        detail::StageLayered(context,mapping,QephFamily,q,qa,3,buffers.Staging(),buffers.flags);
        detail::StageLayered(context,mapping,T3Family,t,ta,1,buffers.Staging(),buffers.flags);
        detail::StageQbat(context,mapping,b,ba,1,buffers.Staging(),buffers.flags);
    }
};
} // namespace crash::output::physical_frames::test
