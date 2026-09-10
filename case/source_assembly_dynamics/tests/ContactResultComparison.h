#pragma once
#include "lib_src/collision/NodalWallContact.h"
#include <array>
#include <cstddef>
#include <type_traits>

namespace crash::cases::source_assembly_dynamics::test {
// Same field inventory as TL NodalWallOwnerFixture::Same, with bit comparison
// (including signed zero) and separate evidence for C++ object padding.
struct ContactByteDifference {
    std::size_t field_bytes=0,padding_bytes=0,first_field_offset=SIZE_MAX,first_padding_offset=SIZE_MAX;
    const char* first_field="none";
};
static_assert(sizeof(tlfea::contact::Q4CertifiedIntegral)==4*sizeof(double));
static_assert(sizeof(tlfea::contact::Vec3)==3*sizeof(double));
template<class T> class ContactFields {
  public:
    ContactFields(const T& a,const T& b):a_(reinterpret_cast<const unsigned char*>(&a)),
        b_(reinterpret_cast<const unsigned char*>(&b)) {static_assert(std::is_standard_layout_v<T>);}
    template<class F> void Field(const char* name,const F& field) {
        const auto offset=reinterpret_cast<const unsigned char*>(&field)-a_;
        for(std::size_t j=0;j<sizeof(F);++j) {
            const auto i=static_cast<std::size_t>(offset)+j;named_[i]=true;
            if(a_[i]!=b_[i]) {
                if(!difference_.field_bytes) {difference_.first_field=name;difference_.first_field_offset=i;}
                ++difference_.field_bytes;
            }
        }
    }
    ContactByteDifference Finish() {
        for(std::size_t i=0;i<sizeof(T);++i)if(!named_[i]&&a_[i]!=b_[i]) {
            if(!difference_.padding_bytes)difference_.first_padding_offset=i;
            ++difference_.padding_bytes;
        }
        return difference_;
    }
  private:
    const unsigned char *a_,*b_;
    std::array<bool,sizeof(T)> named_{};
    ContactByteDifference difference_;
};
inline ContactByteDifference ContactDifference(const tlfea::contact::NodalWallPointResult& a,
                                               const tlfea::contact::NodalWallPointResult& b) {
    ContactFields<tlfea::contact::NodalWallPointResult> compare(a,b);
#define ROBO_CONTACT_FIELD(field) compare.Field(#field,a.field)
    ROBO_CONTACT_FIELD(force);ROBO_CONTACT_FIELD(potential);ROBO_CONTACT_FIELD(stiffness);
    ROBO_CONTACT_FIELD(force_world);ROBO_CONTACT_FIELD(wall_point);ROBO_CONTACT_FIELD(wall_reaction);ROBO_CONTACT_FIELD(wall_moment);
    ROBO_CONTACT_FIELD(surface_power);ROBO_CONTACT_FIELD(local_velocity_first_timestep);
    ROBO_CONTACT_FIELD(row.count);ROBO_CONTACT_FIELD(row.nodes);ROBO_CONTACT_FIELD(row.stiffness);ROBO_CONTACT_FIELD(row.damping);
    ROBO_CONTACT_FIELD(row.base_epoch);ROBO_CONTACT_FIELD(row.attempt);ROBO_CONTACT_FIELD(row.valid);
    ROBO_CONTACT_FIELD(base_epoch);ROBO_CONTACT_FIELD(attempt);ROBO_CONTACT_FIELD(node);
    ROBO_CONTACT_FIELD(fixed);ROBO_CONTACT_FIELD(touching_or_penetrating);ROBO_CONTACT_FIELD(valid);
#undef ROBO_CONTACT_FIELD
    return compare.Finish();
}
inline ContactByteDifference ContactDifference(const tlfea::contact::NodalWallParentResult& a,
                                               const tlfea::contact::NodalWallParentResult& b) {
    ContactFields<tlfea::contact::NodalWallParentResult> compare(a,b);
#define ROBO_CONTACT_FIELD(field) compare.Field(#field,a.field)
    ROBO_CONTACT_FIELD(force);ROBO_CONTACT_FIELD(resultant);ROBO_CONTACT_FIELD(potential);
    ROBO_CONTACT_FIELD(parent_element_id);ROBO_CONTACT_FIELD(feature_id);ROBO_CONTACT_FIELD(parent_face_id);
    ROBO_CONTACT_FIELD(arity);ROBO_CONTACT_FIELD(family);ROBO_CONTACT_FIELD(valid);
#undef ROBO_CONTACT_FIELD
    return compare.Finish();
}
} // namespace crash::cases::source_assembly_dynamics::test
