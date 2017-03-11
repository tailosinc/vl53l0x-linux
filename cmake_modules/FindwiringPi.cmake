# - Try to find WiringPi
# Once done, this will define
#
#  WIRINGPI_FOUND - system has wiringPi
#  WIRINGPI_INCLUDE_DIRS - the wiringPi include directories
#  WIRINGPI_LIBRARIES - link these to use wiringPi

include (cmake_modules/LibFindMacros.cmake)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules (WIRINGPI_PKGCONF wiringpi-odroid)

# Include dir
find_path (WIRINGPI_INCLUDE_DIR
	NAMES wiringPi.h
	PATHS
		${WIRINGPI_PKGCONF_INCLUDE_DIRS}
		/usr/include
		/usr/include/wiringPi
)

# Finally the library itself
find_library (WIRINGPI_LIBRARY
	NAMES wiringPi
	PATHS
		${WIRINGPI_PKGCONF_LIBRARY_DIRS}
		/usr/lib
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set (WIRINGPI_PROCESS_INCLUDES WIRINGPI_INCLUDE_DIR)
set (WIRINGPI_PROCESS_LIBS WIRINGPI_LIBRARY)
libfind_process (WIRINGPI)
