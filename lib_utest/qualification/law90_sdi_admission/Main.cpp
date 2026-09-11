// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Observation.h"
#include <dynamain.h>
#include <radiossblk.h>
#include <dyna2rad.h>
#include <boost/version.hpp>
#include <fstream>
#include <iostream>
#include <memory>

int main(int argc, char** argv) {
  try {
    law90_sdi::Require(argc == 4, "usage: law90_sdi_native input.key export.rad report.json");
    std::unique_ptr<sdi::ModelViewEdit> source(DynakeyReadModel(argv[1]));
    law90_sdi::Require(bool(source), "native LS reader returned no model");
    auto original = law90_sdi::Source(*source);
    RadiossblkSetUserProfileVersion(2026);
    std::unique_ptr<sdi::ModelViewEdit> converted(RadiossblkNewModel());
    law90_sdi::Require(bool(converted), "native target model allocation failed");
    sdiString name = "original_radiator_sdi";
    sdiD2R::DynaToRad converter(source.get(), converted.get(), name);
    converter.CallConvert();
    auto direct = law90_sdi::Target(*converted);
    law90_sdi::Export(*converted, argv[2]);
    std::unique_ptr<sdi::ModelViewEdit> reread(RadiossblkReadModel(argv[2]));
    law90_sdi::Require(bool(reread), "native exported model read failed");
    auto exported = law90_sdi::Target(*reread);
    std::ofstream report(argv[3]);
    law90_sdi::Require(bool(report), "observation report open failed");
    report << std::boolalpha << "{\"schema\":\"law90.native_sdi_observation.v1\","
           << "\"boost_version\":" << BOOST_VERSION << ",\"export_format\":2026,"
           << "\"source_fields\":";
    law90_sdi::WriteFields(report, original);
    report << ",\"direct\":"; law90_sdi::WriteObservation(report, direct);
    report << ",\"exported_reread\":"; law90_sdi::WriteObservation(report, exported);
    report << "}\n"; report.close();
    law90_sdi::Require(bool(report), "observation report write failed");
    return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; }
  catch (const char* e) { std::cerr << "native reader: " << e << '\n'; }
  catch (...) { std::cerr << "native reader unknown exception\n"; }
  return 1;
}
