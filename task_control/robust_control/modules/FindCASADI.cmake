find_path(CASADI_INCLUDE_DIR
  casadi/casadi.hpp
  HINTS $ENV{CASADI_PREFIX}/include $ENV{CASADI_INSTALL_INCLUDE}
)

if(CASADI_INCLUDE_DIR)
  set(CASADI_INCLUDE_DIR ${CASADI_INCLUDE_DIR} ${CASADI_INCLUDE_DIR}/casadi)
  set(CASADI_FOUND_INCLUDE TRUE)
  message(STATUS "Found CasADi include dir: ${CASADI_INCLUDE_DIR}")
else()
  message(STATUS "Could not find CasADi include dir")
endif()

find_library(CASADI_LIBRARY
    NAMES casadi
    HINTS ${CASADI_INCLUDE_LIB} $ENV{CASADI_PREFIX}/lib $ENV{CASADI_INSTALL_LIB})
if(CASADI_LIBRARY)
    set(CASADI_LIBRARIES ${CASADI_LIBRARY})
endif()

if(CASADI_LIBRARIES)
  message(STATUS "Found CasADi libs: ${CASADI_LIBRARIES}")
else()
  message(STATUS "Could not find CasADi libs")
endif()

if(CASADI_FOUND_INCLUDE AND CASADI_LIBRARIES)
  set(CASADI_FOUND TRUE)
endif()