#pragma once
#include "output/full_shell/FullShellVisualizationRecords.h"
#include "output/BoundedArrayJson.h"
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <unistd.h>

namespace crash::output::full_shell::test {
struct Directory {
    Directory() {
        std::string pattern=(std::filesystem::temp_directory_path()/"robo-full-record-XXXXXX").string();
        const auto* result=mkdtemp(pattern.data());if(!result)throw std::runtime_error("Temporary directory failed");
        path=result;
    }
    ~Directory(){std::error_code ignored;std::filesystem::remove_all(path,ignored);}
    std::filesystem::path path;
};
inline Identity Id() {return {1,2,3,4,5,6,13175122,std::string(64,'a'),std::string(64,'b')};}
inline std::vector<ParentPoints> Parents() {
    return {{71,2000137,2,1,3,PlasticField::NativeEquivalentPlasticStrain},
        {72,2000462,9,2,1,PlasticField::NativeEquivalentPlasticStrain},
        {73,2000100,2,3,3,PlasticField::NotApplicable},
        {74,2000200,16,4,3,PlasticField::Unavailable}};
}
inline Context MakeContext() {const auto p=Parents();return Context::Create(Id(),4,p.data(),p.size(),.125);}
inline FrameRecord Frame(bool initial=false) {
    return {initial?FrameStamp{}:FrameStamp{2,1,4,.25,.125,.1875,.125},
        {0.,-0.,1.,2.,3.,4.,5.,6.,7.,8.,9.,10.},{0.,.1,.3,.2}};
}
inline FrameInput View(const FrameRecord& f) {
    return {f.stamp,f.position_xyz.data(),f.position_xyz.size(),f.plastic_points.data(),f.plastic_points.size()};
}
inline RecordFile Rewrite(const std::filesystem::path& root,const RecordFile& file,const Document& document) {
    std::filesystem::remove(root/file.file);WriteJson(root/file.file,document);
    const auto bytes=ReadBounded(root/file.file,FrameMetadataByteCap);
    return {file.file,Sha256(bytes),bytes.size()};
}
inline void Overwrite(const std::filesystem::path& path,const std::string& bytes) {
    std::ofstream f(path,std::ios::binary|std::ios::trunc);f.write(bytes.data(),bytes.size());f.close();
}
} // namespace crash::output::full_shell::test
