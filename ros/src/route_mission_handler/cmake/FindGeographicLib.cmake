find_library(GeographicLib_LIBRARIES Geographic)
find_path(GeographicLib_INCLUDE_DIRS "GeographicLib/Config.h")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GeographicLib DEFAULT_MSG
    GeographicLib_LIBRARIES
    GeographicLib_INCLUDE_DIRS)
mark_as_advanced(
    GeographicLib_LIBRARIES
    GeographicLib_INCLUDE_DIRS)
