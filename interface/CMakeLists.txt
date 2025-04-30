set(BOT_LIB_NAME ${PROJECT_NAME}_interface)
file(GLOB_RECURSE LibSource src/*.cpp)
add_library(${BOT_LIB_NAME} ${COMPILE_TYPE}
        ${LibSource}
)
set_target_properties(${BOT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_include_directories(${BOT_LIB_NAME} PRIVATE include)

target_link_libraries(${BOT_LIB_NAME} pthread ${PROJECT_NAME} )
set_target_properties(${BOT_LIB_NAME} PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION ${PROJECT_SO_VERSION})

if (COMPILE_${PROJECT_NAME}_PYBINDING)
    # Include PythonSetup
    include(PythonSetup)
    # add pybind11
    add_subdirectory(pybind11)
    # Find Python
    find_package(Python COMPONENTS Interpreter Development REQUIRED)

    # Add the module using pybind11
    pybind11_add_module(${PROJECT_NAME}_interface_py interface_py.cpp)
    add_dependencies(${PROJECT_NAME}_interface_py ${PROJECT_NAME}_interface)
    # Link libraries
    target_link_libraries(${PROJECT_NAME}_interface_py PRIVATE ${PROJECT_NAME}_interface)

    #include
    target_include_directories(${PROJECT_NAME}_interface_py PRIVATE include)

    # Set properties for the library
    set_target_properties(${PROJECT_NAME}_interface_py PROPERTIES
            OUTPUT_NAME "${PROJECT_NAME}_interface_py"
            PREFIX ""
            SUFFIX ".so"
    )

    # Install the shared object (.so) target directly to the Python package directory
    install(TARGETS ${PROJECT_NAME}_interface_py
            LIBRARY DESTINATION lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages)
endif ()