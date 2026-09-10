#pragma once
#include "../FullShellFrameGeometry.h"
#include "output/full_shell/static_bundle/tests/ActualMappingSupport.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

namespace crash::visual::full_shell::test {
namespace fsout=output::full_shell;
namespace source=output::full_shell::source;
namespace st=output::full_shell::source::test;
namespace ft=output::full_shell::test;
struct ActualGeometry {
    ActualGeometry() : mapping(Mapping()), context(mapping.MakeFrameContext(fsout::Identity{1,2,3,4,5,6},0x1p-26)) {}
    static source::PreparedSourceMapping Mapping() {
        st::ActualOrder order(st::ActualSource());
        // Formatting-only native declarations, not original material admission.
        order.parents.front().native_points=3;
        order.parents.front().plastic=fsout::PlasticField::NativeEquivalentPlasticStrain;
        order.parents[1].plastic=fsout::PlasticField::NotApplicable;
        order.parents.back().native_points=3;
        order.parents.back().plastic=fsout::PlasticField::NativeEquivalentPlasticStrain;
        return source::PreparedSourceMapping::Prepare(st::ActualSource(),order.View());
    }
    fsout::FrameRecord Frame(bool next=false) const {
        const auto& a=source::FindArray(mapping.source().data(),"node_positions");
        const auto original=output::arrays::Decode<double>(a.descriptor,a.bytes);
        const auto& m=mapping.arrays()[source::detail::NodeCanonical];
        const auto indices=output::arrays::Decode<std::uint32_t>(m.descriptor,m.bytes);
        fsout::FrameRecord result;
        result.position_xyz.reserve(3*indices.size());
        for(auto n:indices)for(unsigned j=0;j<3;++j)
            result.position_xyz.push_back(next&&j==0?original[3*n+j]+.001:original[3*n+j]);
        result.plastic_points.resize(context.points(),0);
        if(next) {
            const double h=context.fixed_dt();
            result.stamp={1,0,7,h,0,.5*h,.5*h};
            for(std::size_t i=0;i<result.plastic_points.size();++i)result.plastic_points[i]=(i+1)*1e-5;
        }
        return result;
    }
    FrameGeometryOptions Options(ReplayColorMode color) const {
        FrameGeometryOptions o;o.geometry=ReplayGeometryLimits::Vehicle();o.colors=color;o.plastic_strain_maximum=.001;return o;
    }
    source::PreparedSourceMapping mapping;
    fsout::Context context;
};
inline const ActualGeometry& Actual() {static const ActualGeometry f;return f;}
inline void ExactPositions(const std::vector<chrono::ChVector3d>& actual,const std::vector<double>& expected) {
    ASSERT_EQ(3*actual.size(),expected.size());
    for(std::size_t n=0;n<actual.size();++n)for(unsigned j=0;j<3;++j)
        ASSERT_EQ(output::Bits(actual[n][j]),output::Bits(expected[3*n+j]));
}
} // namespace crash::visual::full_shell::test
