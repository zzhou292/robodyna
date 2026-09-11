// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <sdiModelView.h>
#include <sdiEntity.h>
#include <array>
#include <iosfwd>
#include <string>
#include <vector>

namespace law90_sdi {
struct Field {
  std::string name;
  bool available = false;
  bool dimension_available = false;
  double value = 0; // Native CPP_GET_FLOATV/INTV missing-field value; availability retained.
  std::array<double, 3> dimension{}; // length, mass, time exponents
};
struct Observation {
  std::vector<Field> fields;
  std::vector<double> curve; // native interleaved x/y, unscaled working quantities
  std::array<double, 33> prepared{};
  unsigned material_id = 0;
  unsigned curve_id = 0;
};
void Require(bool condition, const std::string& message);
sdi::EntityRead Find(sdi::ModelViewRead& model, const char* keyword, unsigned id);
Field Read(const sdi::EntityRead&, const char* name, bool integer = false,
           unsigned index = ~0u);
std::vector<Field> Source(sdi::ModelViewRead&);
Observation Target(sdi::ModelViewRead&);
const Field& Get(const std::vector<Field>&, const char* name);
void Prepare(Observation&);
void WriteFields(std::ostream&, const std::vector<Field>&);
void WriteObservation(std::ostream&, const Observation&);
void Export(sdi::ModelViewEdit&, const std::string& path);
} // namespace law90_sdi
