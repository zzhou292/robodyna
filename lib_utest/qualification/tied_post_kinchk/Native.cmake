include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../tied_shell_classification/Native.cmake")
set(kinchk_native "${CMAKE_CURRENT_LIST_DIR}/native")
target_sources(tied_classification_native PRIVATE
  "${kinchk_native}/Interfaces.F" "${kinchk_native}/Packet.F" "${kinchk_native}/KinChk.F")
target_include_directories(tied_classification_native PRIVATE "${kinchk_native}")
set_source_files_properties("${kinchk_native}/KinChk.F" PROPERTIES COMPILE_DEFINITIONS "my_real=real(kind=8)")
add_library(tied_post_kinchk_oracle INTERFACE)
target_link_libraries(tied_post_kinchk_oracle INTERFACE tied_classification_oracle)
