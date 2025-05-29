#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ur_admittance_controller::ur_admittance_controller" for configuration ""
set_property(TARGET ur_admittance_controller::ur_admittance_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ur_admittance_controller::ur_admittance_controller PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libur_admittance_controller.so"
  IMPORTED_SONAME_NOCONFIG "libur_admittance_controller.so"
  )

list(APPEND _cmake_import_check_targets ur_admittance_controller::ur_admittance_controller )
list(APPEND _cmake_import_check_files_for_ur_admittance_controller::ur_admittance_controller "${_IMPORT_PREFIX}/lib/libur_admittance_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
