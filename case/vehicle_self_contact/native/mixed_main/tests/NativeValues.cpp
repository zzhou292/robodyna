#include "NativeValues.h"
extern "C" void rd_post_gapm_shell_overlay(const double*,const int*,double*);
namespace crash::cases::vehicle_self_contact::native::post_gapm::test {
detail::MainResult Native(const detail::MainPacket& in, const std::array<double,24>& second_coordinates) {
    detail::MainResult out;
    if (in.has_solid) {
        reader_solid_test::Case packet;
        packet.input=in.solid;packet.internal=in.solid.first.face==n::MainFaceKind::Internal;
        packet.second_coordinates=second_coordinates;
        packet.unused_second_controlled_bulk=9e14;
        const auto raw=reader_solid_test::Oracle(packet);
        out.primary=raw.value.stiffness;out.solid_length=raw.value.characteristic_length;
    }
    if (in.has_shell) {
        const auto& s=in.shell;
        const double v[]{s.scale,s.element_thickness,s.property_thickness,s.young,out.primary,out.solid_length,0.,0.,0.};
        const int f[]{s.property_type,s.input_thickness_mode,s.layout==n::ShellLayout::Triangle3?3:4,in.copy_partner?1:0};
        double result[3];rd_post_gapm_shell_overlay(v,f,result);
        out={result[0],result[1],result[2]};
    }
    return out;
}
}
