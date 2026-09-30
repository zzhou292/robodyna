#include "FieldTypes.h"
namespace crash::output::physical_frames {
std::uint32_t Family(tl::fea::ShellBindingFamily family) {
    switch(family) {
    case tl::fea::ShellBindingFamily::Qeph:return QephFamily;
    case tl::fea::ShellBindingFamily::T3:return T3Family;
    case tl::fea::ShellBindingFamily::Qbat:return QbatFamily;
    default:throw std::runtime_error("Unknown native shell family in accepted source mapping");
    }
}
records::PlasticField Plasticity(tl::fea::ShellSectionLaw law) {
    using L=tl::fea::ShellSectionLaw;
    switch(law) {
    case L::LayeredLaw44Nip3:case L::Law44Nip1:case L::Law44QbatFourInPlane:
        return records::PlasticField::NativeEquivalentPlasticStrain;
    case L::LayeredLaw1Nip3:case L::RigidSkin:case L::GlobalLaw1Npt0:return records::PlasticField::NotApplicable;
    default:throw std::runtime_error("Unavailable source role cannot be captured as accepted material");
    }
}
} // namespace crash::output::physical_frames
