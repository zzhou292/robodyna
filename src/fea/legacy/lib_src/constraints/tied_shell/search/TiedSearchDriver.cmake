include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/TiedSearchDriverValues.cmake")
include("${tl_driver_root}/lib_src/collision/HydroelasticBroadphase.cmake")
add_library(tl_tied_search_driver STATIC "${CMAKE_CURRENT_LIST_DIR}/TiedSearchDriver.cu")
target_link_libraries(tl_tied_search_driver PUBLIC tl_tied_search_driver_values tl_collision_broadphase)
set_target_properties(tl_tied_search_driver PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED ON)
target_compile_options(tl_tied_search_driver PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
