
#include "chrono/serialization/ChArchive.h"

#include <stdexcept>

namespace chrono {

ChArchive::VersionNameScope::VersionNameScope(ChArchive& archive, std::type_index type, const char* name)
    : owner(archive), previous_type(archive.version_name_type) {
    if (!name || !*name)
        throw std::invalid_argument("An explicit archive version identity must not be empty");
    // Allocate before mutating the owner; all subsequent swaps are nonthrowing.
    std::string explicit_name(name);
    previous_name.swap(owner.version_name_override);
    explicit_name.swap(owner.version_name_override);
    owner.version_name_type = type;
}

ChArchive::VersionNameScope::~VersionNameScope() noexcept {
    previous_name.swap(owner.version_name_override);
    owner.version_name_type = previous_type;
}

std::string ChArchive::VersionClassName(std::type_index type) const {
    if (!version_name_override.empty() && version_name_type == type)
        return version_name_override;
    return ChClassFactory::IsClassRegistered(type) ? ChClassFactory::GetClassTagName(type) : std::string(type.name());
}

ChArchive::ChArchive() {
    use_versions = true;
    cluster_class_versions = true;
}

void ChArchive::SetUseVersions(bool muse) {
    this->use_versions = muse;
}

void ChArchive::SetClusterClassVersions(bool mcl) {
    this->cluster_class_versions = mcl;
}

void ChArchiveOut::PutPointer(void* object, bool& already_stored, size_t& obj_ID) {
    if (this->internal_ptr_id.find(static_cast<void*>(object)) != this->internal_ptr_id.end()) {
        already_stored = true;
        obj_ID = internal_ptr_id[static_cast<void*>(object)];
        return;
    }

    // wasn't in list.. add to it
    ++currentID;
    obj_ID = currentID;
    internal_ptr_id[static_cast<void*>(object)] = obj_ID;
    already_stored = false;
    return;
}

void ChArchiveOut::out_version(int mver, const std::type_index mtypeid) {
    if (use_versions) {
        std::string class_name = this->VersionClassName(mtypeid);
        // avoid issues with XML format
        std::replace(class_name.begin(), class_name.end(), '<', '_');
        std::replace(class_name.begin(), class_name.end(), '>', '_');
        std::replace(class_name.begin(), class_name.end(), ' ', '_');
        // avoid issues with FMU variable naming format
        std::replace(class_name.begin(), class_name.end(), ':', '_');
        this->out(ChNameValue<int>(("_version_" + class_name).c_str(), mver, 0,
                                 ChCausalityType::local, ChVariabilityType::constant));
    }
}

ChArchiveIn::ChArchiveIn() : can_tolerate_missing_tokens(false) {
    internal_ptr_id.clear();
    internal_ptr_id[nullptr] = 0;  // null pointer -> ID=0.
    internal_id_ptr.clear();
    internal_id_ptr[0] = nullptr;  // ID=0 -> null pointer.
}

int ChArchiveIn::in_version(const std::type_index mtypeid) {
    int mver;
    std::string class_name = this->VersionClassName(mtypeid);
    // avoid issues with XML format
    std::replace(class_name.begin(), class_name.end(), '<', '_');
    std::replace(class_name.begin(), class_name.end(), '>', '_');
    std::replace(class_name.begin(), class_name.end(), ' ', '_');
    // avoid issues with FMU variable naming format
    std::replace(class_name.begin(), class_name.end(), ':', '_');
    this->in(ChNameValue<int>(("_version_" + class_name).c_str(), mver));
    return mver;
}

}  // end namespace chrono
