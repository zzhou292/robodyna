#include "tests/serialization_compat/rename_probe/LegacyTypes.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>

#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono/serialization/ChArchiveXML.h"

using chrono::make_ChNameValue;

namespace {
template <class Out, class In>
void WriteAndCheck(const std::filesystem::path& path) {
    std::ostringstream stream(std::ios::out | std::ios::binary);
    auto object = std::make_shared<chrono::ChRenameProbe>();
    std::shared_ptr<chrono::ChRenameProbeBase> base_alias = object;
    auto repeated = object;
    std::shared_ptr<chrono::ChRenameProbe> null_object;
    if (static_cast<void*>(object.get()) == static_cast<void*>(base_alias.get()))
        throw std::runtime_error("probe does not exercise an adjusted secondary base pointer");
    {
        Out archive(stream);
        archive << CHNVP(object) << CHNVP(base_alias) << CHNVP(repeated) << CHNVP(null_object);
    }
    const auto bytes = stream.str();
    std::istringstream input(bytes, std::ios::in | std::ios::binary);
    object.reset();
    base_alias.reset();
    repeated.reset();
    {
        In archive(input);
        archive >> CHNVP(object) >> CHNVP(base_alias) >> CHNVP(repeated) >> CHNVP(null_object);
    }
    if (!object || object->value != 2.5 || object->base_value != 17 || object->Kind() != 41 ||
        object->observed_version != 5 || object->observed_base_version != 3 || null_object ||
        object != repeated || base_alias.get() != static_cast<chrono::ChRenameProbeBase*>(object.get()) ||
        object.owner_before(base_alias) || base_alias.owner_before(object))
        throw std::runtime_error("legacy probe archive round trip changed values, versions or shared graph");
    std::ofstream file(path, std::ios::binary);
    file.write(bytes.data(), bytes.size());
    file.close();
    if (!file)
        throw std::runtime_error("cannot write legacy probe archive");
}
}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 2)
            throw std::runtime_error("usage: write_legacy_probe NEW_OUTPUT_DIRECTORY");
        const std::filesystem::path output(argv[1]);
        if (!std::filesystem::create_directory(output))
            throw std::runtime_error("legacy probe output already exists; refusing overwrite");
        WriteAndCheck<chrono::ChArchiveOutJSON, chrono::ChArchiveInJSON>(output / "baseline.json");
        WriteAndCheck<chrono::ChArchiveOutXML, chrono::ChArchiveInXML>(output / "baseline.xml");
        WriteAndCheck<chrono::ChArchiveOutBinary, chrono::ChArchiveInBinary>(output / "baseline.bin");
        {
            std::ofstream stream(output / "producer.json");
            chrono::ChArchiveOutJSON archive(stream);
            std::string compiler = __VERSION__;
            auto language = static_cast<unsigned long long>(__cplusplus);
            auto pointer_bytes = static_cast<unsigned long long>(sizeof(void*));
            auto size_t_bytes = static_cast<unsigned long long>(sizeof(std::size_t));
            std::string base_rtti = typeid(chrono::ChRenameProbeBase).name();
            std::string derived_rtti = typeid(chrono::ChRenameProbe).name();
            archive << CHNVP(compiler) << CHNVP(language) << CHNVP(pointer_bytes) << CHNVP(size_t_bytes)
                    << CHNVP(base_rtti) << CHNVP(derived_rtti);
        }
        std::cout << "Wrote three authentic legacy probe archives; values, versions and shared graph passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
