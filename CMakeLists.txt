cmake_minimum_required(VERSION 3.26)

if (NOT DEFINED CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()
set(CMAKE_C_STANDARD 99)

set(CMAKE_CXX_EXTENSIONS OFF)

set(PROJECT_VERSION 1.2.1)
set(PROJECT_SO_VERSION 1)
# Set the author and email
set(AUTHOR "YX.E.Z And SJ.Fang")
set(AUTHOR_EMAIL "yixuanzhou@sjtu.edu.cn And fang20021005@sjtu.edu.cn")
set(PACKAGE_DESCRIPTION "Universal Tele-operation Trajectory Generation")

# Set the project name and language
project(
        UTTG
        VERSION ${PROJECT_VERSION}
        DESCRIPTION ${PACKAGE_DESCRIPTION}
        LANGUAGES CXX C)


option(ENABLE_WARNINGS "Enable project warnings" ON)
option(ENABLE_SANITIZER "Enable address and undefined behavior sanitizer" OFF)
option(ENABLE_TEST "Enable test " OFF)
option(ENABLE_DOXYGEN "Enable doxygen" OFF)
option(ENABLE_STATIC "Build with static library" OFF)
option(COMPILE_${PROJECT_NAME}_PYBINDING "Compile pybind " OFF)

# Set up modules path and include necessary scripts from cmake_template
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(FetchContent)

# Include standard project settings
include(StandardProjectSettings)

if (USE_STATIC_LIB)
    SET(COMPILE_TYPE "STATIC")
else ()
    SET(COMPILE_TYPE "SHARED")
    set(CMAKE_POSITION_INDEPENDENT_CODE ON) # -fPIC
endif ()

# find package area
include(Utilities)

# find IRMV packages
set(THIS_PACKAGE_IRMV_DEPENDS
        "irmv_core 0.2"
        "imc 1.0"
)
prepare_irmv_dependencies("${THIS_PACKAGE_IRMV_DEPENDS}" IRMV_PACKAGE_DEPENDS)

find_package(Eigen3 3.1 REQUIRED)
find_package(yaml-cpp 0.6 REQUIRED)
set(THIS_PACKAGE_DEPENDS
        "Eigen3 3.1"
        "yaml-cpp 0.6"
)

set(THIS_PACKAGE_BINARY_DEPENDS
        "libeigen3-dev 3.1"
        "libyaml-cpp-dev 0.6"
)
prepare_debian_dependencies("${THIS_PACKAGE_BINARY_DEPENDS}" DEBIAN_PACKAGE_DEPENDS)
add_definitions(-DPLOG_CAPTURE_FILE)
add_definitions(-DFMT_HEADER_ONLY)
add_definitions(-DRESOURCE_PATH="${CMAKE_SOURCE_DIR}/resources")
add_definitions(-DCONFIG_MY_PATH="${CMAKE_SOURCE_DIR}/config")

set(THIS_PACKAGE_INCLUDE_DIRS
        include
        interface/include
)

if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
else ()
    set(${PROJECT_NAME}_INCLUDE_DIRS_CHILD )
    foreach(DIR ${THIS_PACKAGE_INCLUDE_DIRS})
        set(${PROJECT_NAME}_DIR "${CMAKE_CURRENT_SOURCE_DIR}/${DIR}")
        list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS_CHILD ${${PROJECT_NAME}_DIR})
    endforeach()
    set(${PROJECT_NAME}_INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDE_DIRS_CHILD} PARENT_SCOPE)
endif()

include_directories(
        SYSTEM
        ${EIGEN3_INCLUDE_DIRS}
        ${THIS_PACKAGE_INCLUDE_DIRS}
        ${irmv_core_INCLUDE_DIRS}
        ${imc_INCLUDE_DIRS}
)

if (ENABLE_TEST)
    add_subdirectory(test)
endif ()

add_library(${PROJECT_NAME}
        ${COMPILE_TYPE}
        src/ArmServoModeBase.cpp
        src/ArmServoModeTrajectory.cpp
        src/ArmServoModeDirect.cpp
        src/ArmServoModeInterpolation.cpp
        src/ArmServoModeLinear.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC ${imc_LIBRARIES})

add_subdirectory(interface)

include(PackageProject)
myproject_package_project(
        NAME ${PROJECT_NAME}
        VERSION ${PROJECT_VERSION}
        TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_interface
        PUBLIC_INCLUDES ${THIS_PACKAGE_INCLUDE_DIRS}
        PUBLIC_DEPENDENCIES ${THIS_PACKAGE_IRMV_DEPENDS} ${THIS_PACKAGE_DEPENDS}
)

set(CPACK_PACKAGE_FILE_NAME
        "${CMAKE_PROJECT_NAME}-${CMAKE_PROJECT_VERSION}-${GIT_SHORT_SHA}-${CMAKE_SYSTEM_NAME}-${CMAKE_BUILD_TYPE}-${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION}"
)

set(CPACK_GENERATOR "DEB")

set(CPACK_PACKAGE_NAME "${CMAKE_PROJECT_NAME}")
set(CPACK_PACKAGE_VERSION "${CMAKE_PROJECT_VERSION}")
set(CPACK_PACKAGE_DIRECTORY ${CMAKE_BINARY_DIR})
set(CPACK_DEBIAN_PACKAGE_MAINTAINER ${AUTHOR})

# Use a valid Debian architecture name
if ("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
elseif ("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
else ()
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE ${CMAKE_SYSTEM_PROCESSOR})
endif ()

set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.28), libstdc++6 (>= 9.4), python3 (>= 3.8), ${DEBIAN_PACKAGE_DEPENDS}, ${IRMV_PACKAGE_DEPENDS}")
include(CPack)

if (ENABLE_DOXYGEN)
    include(Doxygen)
    # Exclude the pybind11 and third_party directories from the documentation
    set(EXCLUDE_PATH_AND_ITEMS
            "${CMAKE_CURRENT_SOURCE_DIR}/interface/pybind11"
            "${CMAKE_CURRENT_SOURCE_DIR}/third_party"
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake-build-debug"
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake-build-release"
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake-build-docker-debug"
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake-build-docker-release"
            "${CMAKE_CURRENT_SOURCE_DIR}/example"
            "${CMAKE_CURRENT_SOURCE_DIR}/setup.py"
    )

    myproject_enable_doxygen("awesome-sidebar" "${EXCLUDE_PATH_AND_ITEMS}")
endif ()