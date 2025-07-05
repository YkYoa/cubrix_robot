#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ar_utils::ar_utils" for configuration ""
set_property(TARGET ar_utils::ar_utils APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ar_utils::ar_utils PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libar_utils.so"
  IMPORTED_SONAME_NOCONFIG "libar_utils.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS ar_utils::ar_utils )
list(APPEND _IMPORT_CHECK_FILES_FOR_ar_utils::ar_utils "${_IMPORT_PREFIX}/lib/libar_utils.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
