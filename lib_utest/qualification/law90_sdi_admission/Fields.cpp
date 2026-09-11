// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Observation.h"
#include <radiossblk.h>
#include <sdiValue.h>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <ostream>
#include <stdexcept>

namespace law90_sdi {
void Require(bool ok, const std::string& message) {
  if (!ok) throw std::runtime_error(message);
}
sdi::EntityRead Find(sdi::ModelViewRead& model, const char* keyword, unsigned id) {
  sdi::HandleRead handle;
  Require(model.FindById(model.GetEntityType(keyword), id, handle) && handle.IsValid(),
          std::string("native entity missing: ") + keyword);
  return sdi::EntityRead(&model, handle);
}
Field Read(const sdi::EntityRead& entity, const char* name, bool integer, unsigned index) {
  Field f; f.name = name;
  f.dimension_available = RadiossblkGetDimensions(entity, name,
      &f.dimension[0], &f.dimension[1], &f.dimension[2]);
  if (integer) {
    sdiValue value; int number = 0;
    f.available = entity.GetValue(sdiIdentifier(name, 0, index), value) && value.GetValue(number);
    if (f.available) f.value = number;
  } else {
    f.available = RadiossblkGetValueDouble(entity, name, &f.value,
        static_cast<double*>(nullptr), static_cast<double*>(nullptr), static_cast<double*>(nullptr), index);
    if (!f.available) f.value = 0;
  }
  Require(std::isfinite(f.value), f.name + " nonfinite field");
  for (double d : f.dimension) Require(std::isfinite(d), f.name + " nonfinite dimension");
  return f;
}
std::vector<Field> Source(sdi::ModelViewRead& model) {
  auto material = Find(model, "*MAT_LOW_DENSITY_FOAM", 2000063);
  std::vector<Field> fields;
  for (const char* name : {"RHO", "E", "HU", "SHAPE", "DAMP", "KCON", "TC", "FAIL"})
    fields.push_back(Read(material, name));
  return fields;
}
const Field& Get(const std::vector<Field>& fields, const char* name) {
  for (const auto& field : fields) if (field.name == name) return field;
  throw std::runtime_error(std::string("observation field missing: ") + name);
}
Observation Target(sdi::ModelViewRead& model) {
  auto material = Find(model, "/MAT", 2000063);
  Require(material.GetKeyword() == "/MAT/LAW90", "converted material is not LAW90");
  Observation out; out.material_id = material.GetId();
  for (const char* name : {"MAT_RHO", "Refer_Rho", "MAT_E0", "MAT_NU", "Fcut",
                           "MAT_SHAPE", "Hys", "MAT_ALPHA", "LSD_MAT83_ED", "LSD_MAT83_TC"})
    out.fields.push_back(Read(material, name));
  for (const char* name : {"NL", "Ismooth", "MAT_TFLAG", "LSD_MAT83_FAIL"})
    out.fields.push_back(Read(material, name, true));
  out.fields.push_back(Read(material, "EpsilondotL", false, 0));
  out.fields.push_back(Read(material, "FscaleL", false, 0));
  // A non-copied source DAMP may not be represented as native material damping.
  out.fields.push_back(Read(material, "DAMP"));
  Require(Get(out.fields, "NL").value == 1, "native curve count is not one");
  sdi::HandleRead curve_handle;
  Require(material.GetEntityHandle(sdiIdentifier("fct_IDL", 0, 0), curve_handle) &&
          curve_handle.IsValid(), "native LAW90 curve reference missing");
  sdi::EntityRead curve(&model, curve_handle); out.curve_id = curve.GetId();
  sdiValue points;
  Require(curve.GetValue(sdiIdentifier("points"), points) && points.GetValue(out.curve),
          "native curve values unavailable");
  Require(out.curve_id == 2100015 && out.curve.size() == 56, "native curve identity/shape");
  for (double v : out.curve) Require(std::isfinite(v), "nonfinite native curve");
  Prepare(out);
  return out;
}
void WriteFields(std::ostream& out, const std::vector<Field>& fields) {
  out << '['; bool first = true;
  for (const auto& f : fields) {
    if (!first) out << ',';
    first = false;
    std::uint64_t bits = 0; std::memcpy(&bits, &f.value, sizeof(bits));
    out << "{\"name\":\"" << f.name << "\",\"available\":" << f.available
        << ",\"effective_native_value\":" << std::setprecision(17) << f.value
        << ",\"binary64_hex\":\"" << std::hex << std::setw(16) << std::setfill('0')
        << bits << std::dec << "\",\"dimension_available\":" << f.dimension_available
        << ",\"dimensions_lmt\":[" << f.dimension[0] << ',' << f.dimension[1]
        << ',' << f.dimension[2] << "]}";
  }
  out << ']';
}
void WriteObservation(std::ostream& out, const Observation& observation) {
  out << "{\"material_id\":" << observation.material_id << ",\"curve_id\":"
      << observation.curve_id << ",\"fields\":";
  WriteFields(out, observation.fields);
  out << ",\"curve_working_xy\":[";
  for (unsigned i = 0; i < observation.curve.size(); ++i) {
    if (i) out << ','; out << std::setprecision(17) << observation.curve[i];
  }
  out << "],\"native_preparation_si\":[";
  for (unsigned i = 0; i < observation.prepared.size(); ++i) {
    if (i) out << ','; out << std::setprecision(17) << observation.prepared[i];
  }
  out << "],\"qualified_positive_hys_profile\":" << (observation.prepared[16] == 2) << '}';
}
} // namespace law90_sdi
