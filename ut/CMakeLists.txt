
include_directories(${CMAKE_SOURCE_DIR}
                    ${CMAKE_SOURCE_DIR}/msgs)

file(GLOB TEST_SOURCES "*.cpp")

find_package(Threads REQUIRED)
foreach(test_src ${TEST_SOURCES})
    get_filename_component(test_name ${test_src} NAME_WE)

    add_executable(${test_name} ${test_src})
    target_link_libraries(${test_name} PRIVATE Threads::Threads)
endforeach()