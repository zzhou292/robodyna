# Existing C2 routine on prescribed saved geometry; no physical state owner.
add_executable(robo-dyna-contact-profile Q4ContactProfile.cu Q4ContactProfileInput.cpp
  Q4ContactProfileReport.cpp q4_contact_profile_main.cpp)
target_link_libraries(robo-dyna-contact-profile PRIVATE robo_dyna_guided_plate_reference
  robo_dyna_canonical_wall_artifacts robo_dyna_artifact_io CUDA::cudart)
target_include_directories(robo-dyna-contact-profile PRIVATE "${CRASH_TL_FEA_SOURCE_DIR}")
set_target_properties(robo-dyna-contact-profile PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(robo-dyna-contact-profile PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
