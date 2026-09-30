// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Observation.h"
#include <sdiSelection.h>
#include <cfgkernel/sdi/sdiModelViewPO.h>
#include <HCDI/hcdi_mec_pre_object.h>
#include <cfgio/MODEL_IO/mec_data_writer.h>
#include <cfgio/MODEL_IO/mec_single_file_writer.h>
#include <cstdio>

namespace law90_sdi {
namespace {
const IMECPreObject& Object(const sdi::EntityRead& entity) {
  auto* object = static_cast<const IMECPreObject*>(entity.GetHandle().GetPointer());
  Require(object != nullptr, "native preobject missing");
  return *object;
}
void Preflight(sdi::ModelViewEdit& model) {
  auto material = Find(model, "/MAT", 2000063);
  for (const auto* child : Object(material).GetSubobject())
    Require(child == nullptr, "active optional material subobject unsupported by scalar export");
  for (const char* field : {"Heat_Inp_opt", "THERM_STRESS", "VISC_PRONY_option"})
    Require(Read(material, field, true).value == 0,
            "active optional material subobject unsupported by scalar export");
  auto curve = Find(model, "/FUNCT", 2100015);
  const auto& children = Object(curve).GetSubobject();
  unsigned moves = 0;
  for (const auto* child : children) {
    if (!child) continue;
    const char* type = child->GetKernelFullType();
    Require(type && std::string(type) == "/SUBOBJECT/MOVE_FUNCT",
            std::string("unknown curve subobject in scalar export: ") + (type ? type : "null"));
    Require(child->GetSubobject().empty(), "nested curve subobject");
    ++moves;
    const char* names[4] = {"A_SCALE_X", "F_SCALE_Y", "A_SHIFT_X", "F_SHIFT_Y"};
    const double expected[4] = {1, 1, 0, 0};
    for (unsigned i = 0; i < 4; ++i) {
      const int index = child->GetIndex(IMECPreObject::ATY_SINGLE, IMECPreObject::VTY_FLOAT, names[i]);
      Require(index >= 0 && child->GetFloatValue(index) == expected[i],
              "nonidentity MOVE_FUNCT unsupported by scalar export");
    }
  }
  Require(moves == 1, "converted identity MOVE_FUNCT receipt missing");
}
}
void Export(sdi::ModelViewEdit& model, const std::string& path) {
  Preflight(model); // reject unsupported scope before creating any output
  FILE* file = std::fopen(path.c_str(), "wx");
  Require(file != nullptr, "create-only native export failed");
  try {
    auto& preobjects = static_cast<sdi::ModelViewPO&>(model);
    // Same complete native main-card writer/configuration as PrintOption.cpp.
    // Its public scanner-free overload intentionally omits subobjects. The only
    // omitted curve operation was authenticated above as exact identity; LAW90
    // optional material operations were explicitly rejected.
    for (const char* keyword : {"/BEGIN", "/FUNCT", "/MAT", "/END"}) {
      sdi::SelectionRead entities(&model, keyword);
      while (entities.Next()) {
        const auto& object = Object(*entities);
        const auto* descriptor = preobjects.GetDescriptor(&object);
        Require(descriptor != nullptr, "native export descriptor missing");
        const auto* format = descriptor->getRadiossFileFormatPtr(FF_D00_2026);
        Require(format != nullptr, "native modern export format missing");
        MECSingleFileWriter filewriter(file, 100);
        MECDataWriter writer(&filewriter, 100);
        writer.setFormatId(FF_D00_2026);
        writer.setCompressDouble(1);
        writer.setRoundDouble(0);
        writer.WriteObjectData(format, object, descriptor);
      }
    }
    const int failed = std::ferror(file);
    const int closed = std::fclose(file); file = nullptr;
    Require(!failed && !closed, "native export write failed");
  } catch (...) { if (file) std::fclose(file); throw; }
}
} // namespace law90_sdi
