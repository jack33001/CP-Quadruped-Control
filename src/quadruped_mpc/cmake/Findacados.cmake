
# Find ACADOS library and include files
#
# Sets:
#  acados_FOUND
#  acados_INCLUDE_DIRS
#  acados_LIBRARIES

# Try to find ACADOS in standard paths or using env variable
if(NOT DEFINED ENV{ACADOS_SOURCE_DIR})
    set(ACADOS_SEARCH_PATHS
        /usr/local
        /usr
        /opt/acados
    )
else()
    set(ACADOS_SEARCH_PATHS $ENV{ACADOS_SOURCE_DIR})
endif()

# Find include directory
find_path(acados_INCLUDE_DIR
    NAMES acados/acados.h
    PATHS ${ACADOS_SEARCH_PATHS}
    PATH_SUFFIXES include
)

# Find libraries
find_library(ACADOS_LIB
    NAMES acados
    PATHS ${ACADOS_SEARCH_PATHS}
    PATH_SUFFIXES lib
)

find_library(HPIPM_LIB
    NAMES hpipm
    PATHS ${ACADOS_SEARCH_PATHS}
    PATH_SUFFIXES lib
)

find_library(BLASFEO_LIB
    NAMES blasfeo
    PATHS ${ACADOS_SEARCH_PATHS}
    PATH_SUFFIXES lib
)

# Set libraries
set(acados_LIBRARIES
    ${ACADOS_LIB}
    ${HPIPM_LIB}
    ${BLASFEO_LIB}
)

# Set include dirs
set(acados_INCLUDE_DIRS
    ${acados_INCLUDE_DIR}
    ${acados_INCLUDE_DIR}/acados
    ${acados_INCLUDE_DIR}/blasfeo/include
    ${acados_INCLUDE_DIR}/hpipm/include
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(acados
    REQUIRED_VARS
        acados_INCLUDE_DIR
        ACADOS_LIB
        HPIPM_LIB
        BLASFEO_LIB
)

mark_as_advanced(
    acados_INCLUDE_DIR
    ACADOS_LIB
    HPIPM_LIB
    BLASFEO_LIB
)