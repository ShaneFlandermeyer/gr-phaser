find_package(PkgConfig)

PKG_CHECK_MODULES(PC_GR_PHASER gnuradio-phaser)

FIND_PATH(
    GR_PHASER_INCLUDE_DIRS
    NAMES gnuradio/phaser/api.h
    HINTS $ENV{PHASER_DIR}/include
        ${PC_PHASER_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GR_PHASER_LIBRARIES
    NAMES gnuradio-phaser
    HINTS $ENV{PHASER_DIR}/lib
        ${PC_PHASER_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/gnuradio-phaserTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_PHASER DEFAULT_MSG GR_PHASER_LIBRARIES GR_PHASER_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_PHASER_LIBRARIES GR_PHASER_INCLUDE_DIRS)
