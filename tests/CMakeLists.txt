include(GoogleTest)

# One executable per file, each registered with CTest.
file(GLOB HWEMU_TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp")
foreach (source ${HWEMU_TEST_SOURCES})
    get_filename_component(name ${source} NAME_WE)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE hwemu hwemu_client GTest::gtest_main)
    target_include_directories(${name} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
    target_compile_definitions(${name} PRIVATE
        HWEMU_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/golden"
        HWEMU_EXAMPLES_DIR="${PROJECT_SOURCE_DIR}/examples"
    )
    gtest_discover_tests(${name} DISCOVERY_TIMEOUT 60 PROPERTIES TIMEOUT 600)
endforeach ()

# The Python module and the pyrealsense2 shim, end to end.
if (TARGET _hwemu)
    add_test(NAME python_end_to_end
        COMMAND ${Python3_EXECUTABLE} -m unittest discover -v -s . -p "test_*.py"
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/python")
    set_tests_properties(python_end_to_end PROPERTIES
        ENVIRONMENT "PYTHONPATH=${PROJECT_SOURCE_DIR}/python/shims:${PROJECT_SOURCE_DIR}/python"
        TIMEOUT 120)
endif ()
