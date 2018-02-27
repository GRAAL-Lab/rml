# RML: Robotic Matematical Library

# At LEAST 2.8 but newer is better
cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

# Must use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)

# Define library. Only source files here!
project(RML_project VERSION 0.1 LANGUAGES CXX)

add_library(rml SHARED
    src/Functions.cc
    src/PseudoInverse.cc
    src/PseudoInverse_Internal.cc
    src/SVD.cc
    src/SVD_Internal.cc
    src/ArmModel.cc
    src/VehicleModel.cc
    src/RobotModel.cc
    src/Types.cc
    src/RobotLink.cc
)

#add_library(youBotModel include/test/youbot_armmodel.cc
#)

set_target_properties(rml PROPERTIES LINKER_LANGUAGE CXX)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(rml PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# If we have compiler requirements for this library, list them
# here
target_compile_features(rml
    PUBLIC
        cxx_auto_type
        cxx_range_for
        cxx_variadic_templates
)

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

# Every library has unit tests, of course
add_executable(rml_test test/rml_test.cpp)

add_executable(rml_test_model test/rml_test_model.cpp
    test/youbot_armmodel.cc
    test/baxterLeft_armmodel.cc
)

target_link_libraries(rml_test rml)
target_link_libraries(rml_test_model rml)

add_test(rml_test rml_test)
add_test(rml_test_model rml_test_model)
