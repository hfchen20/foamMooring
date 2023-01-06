#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "MoorDyn::moordyn" for configuration "Release"
set_property(TARGET MoorDyn::moordyn APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(MoorDyn::moordyn PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libmoordyn.so.2.0"
  IMPORTED_SONAME_RELEASE "libmoordyn.so.2"
  )

list(APPEND _IMPORT_CHECK_TARGETS MoorDyn::moordyn )
list(APPEND _IMPORT_CHECK_FILES_FOR_MoorDyn::moordyn "${_IMPORT_PREFIX}/lib/libmoordyn.so.2.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
