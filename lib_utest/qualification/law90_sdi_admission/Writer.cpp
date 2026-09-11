// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Observation.h"
#include <sdiSelection.h>
#include <cstdio>

// Native PrintOption.cpp's sole external owner lookup. This is storage only;
// all descriptor selection/default/number formatting is in the unchanged donor.
namespace { sdi::ModelViewEdit* active_model = nullptr; }
sdi::ModelViewEdit* Get_ModelViewSDI() { return active_model; }
void PrintEntity(FILE*, sdi::ModelViewRead*, const sdi::EntityRead&, int*);

namespace law90_sdi {
void Export(sdi::ModelViewEdit& model, const std::string& path) {
  FILE* file = std::fopen(path.c_str(), "wx");
  Require(file != nullptr, "create-only native export failed");
  active_model = &model; int is_dyna = 0;
  try {
    for (const char* keyword : {"/BEGIN", "/FUNCT", "/MAT", "/PROP", "/PART", "/DEF_SOLID", "/END"}) {
      sdi::SelectionRead entities(&model, keyword);
      while (entities.Next()) PrintEntity(file, &model, *entities, &is_dyna);
    }
    active_model = nullptr;
    const int failed = std::ferror(file);
    const int closed = std::fclose(file); file = nullptr;
    Require(!failed && !closed, "native export write failed");
  } catch (...) { active_model = nullptr; if (file) std::fclose(file); throw; }
}
} // namespace law90_sdi
