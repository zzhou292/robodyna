#pragma once
#include "../NativeSceneRun.h"
#include "output/full_shell/tests/TestSupport.h"
#include <cstdlib>
namespace crash::cases::native_scene::test {
namespace ft=output::full_shell::test;
inline modelio::native_scene::DeclaredSource Declared(const char* environment="ROBO_DYNA_NATIVE_SCENE_EXPORT") {
    const char* p=std::getenv(environment);if(!p)throw std::runtime_error("Explicit source fixture required");
    return modelio::native_scene::DeclaredSource::Read(p,output::Sha256(output::ReadBounded(p,4u<<20)));
}
struct SourceFixture {
    ft::Directory directory;
    PhysicalSource physical;
    ContactSelection contact;
    ArchiveSource archive;
    explicit SourceFixture(const char* environment="ROBO_DYNA_NATIVE_SCENE_EXPORT")
      :physical(PhysicalSource::Prepare(Declared(environment),771)),
       contact(ContactSelection::Prepare(physical,{1,1,1})),archive(ArchiveSource::Write(physical,directory.path)){}
    DynamicsConfig config() const {DynamicsConfig c;c.configuration=901;c.qualification=902;c.fixed_dt=3e-7;return c;}
    output::full_shell::Identity identity() const {output::full_shell::Identity i;i.run=903;return i;}
    NativeSceneDynamics MakeDynamics() const{return NativeSceneDynamics::Prepare(contact,config());}
    auto MakeCapture(NativeSceneDynamics& d) const{return d.MakeCapture(archive.mapping(),identity());}
};
}
