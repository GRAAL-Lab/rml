# RML: Robotic Matematical Library

# At LEAST 2.8 but newer is better
cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

# Must use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Define library. Only source files here!
project(RML_project VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CONFIGURATION_TYPES "Debug" CACHE STRING "" FORCE)

option(BUILD_TESTS "Compile tests" OFF)
option(BUILD_TESTS_DEVEL "Compile tests devel branch" ON)

add_library(rml SHARED
    src/Vector6d.cc
    src/Vector9d.cc
    src/RotMatrix.cc
    src/TransfMatrix.cc
    src/EulerRPY.cc
    src/Functions.cc
    src/PseudoInverse.cc
    src/PseudoInverse_Internal.cc
    src/SVD.cc
    src/SVD_Internal.cc
    src/RobotLink.cc
    src/ArmModel.cc
    src/VehicleModel.cc
    src/RobotModel.cc
#    src/NewtonEuler.cc
)

set_target_properties(rml PROPERTIES LINKER_LANGUAGE CXX)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(rml PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/rml>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# If we have compiler requirements for this library, list them
# here
target_compile_features(rml
    PUBLIC
        cxx_auto_type
        cxx_range_for
        cxx_variadic_templates)

# Depend on a library that we defined in the top-level file
#target_link_libraries(lib
#    boost
#    MyOtherLibrary)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS rml EXPORT rmlConfig
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/rml DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT rmlConfig DESTINATION share/rml/cmake)

# This makes the project importable from the build directory
export(TARGETS rml FILE rmlConfig.cmake)


if(BUILD_TESTS)
    # Every library has unit tests, of course
    add_executable(rml_test test/rml_test.cpp)

    add_executable(rml_test_model test/rml_test_model.cpp
        test/baxterLeft_armmodel.cc
        test/youbot_armmodel.cc
        test/youbot_vehiclemodel.cc)

    #add_executable(rml_test_newtoneuler test/rml_test_newtoneuler.cpp
    #    test/youbot_armmodel.cc
    #    test/youbot_vehiclemodel.cc
    #    test/twolinks_armmodel.cc)

    #add_executable(robot_simulator test/robot_simulator.cpp
    #    test/youbot_armmodel.cc
    #    test/youbot_vehiclemodel.cc
    #   test/twolinks_armmodel.cc
    #   test/OdeIntSimulator.cpp)

    target_link_libraries(rml_test rml)
    target_link_libraries(rml_test_model rml)
    #target_link_libraries(rml_test_newtoneuler rml)
    #target_link_libraries(robot_simulator rml)

    add_test(rml_test rml_test)
    add_test(rml_test_model rml_test_model)
    #add_test(rml_test_newtoneuler rml_test_newtoneuler)
    #add_test(robot_simulator robot_simulator)
endif(BUILD_TESTS)


if(BUILD_TESTS_DEVEL)
    add_executable(frameTest test/frameidentificationtest.cpp
                             test/youbot_armmodel.cc
                             test/twolinks_armmodel.cc)

    target_link_libraries(frameTest rml)

    add_test(rml_test frame_test)
endif(BUILD_TESTS_DEVEL)
