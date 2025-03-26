###########################################################################
# CMakeLists.txt generated for component run_tag_test_chipotle
# Product type: STATIC library
###########################################################################
cmake_minimum_required(VERSION 3.12)
project(run_tag_test_chipotle)

# Propagate the CMAKE_EXPORT_COMPILE_COMMANDS variable from the 
# environment if it is defined as an environment variable, but not as a 
# CMake variable. This is to work around a bug in CMake 3.19 when the 
# "NMake Makefiles" generator is selected.
if(DEFINED ENV{CMAKE_EXPORT_COMPILE_COMMANDS} AND NOT DEFINED CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_EXPORT_COMPILE_COMMANDS $ENV{CMAKE_EXPORT_COMPILE_COMMANDS})
endif()


###########################################################################
## Path variables
###########################################################################
# Derive an absolute path to the code generation anchor folder.
get_filename_component(START_DIR ../../.. ABSOLUTE)

# Special directories defined by using CACHE variables can be overridden 
# by setting the variable from the command line, e.g.,
# 
# 	cmake . -DMATLAB_ROOT=/path/to/another/matlab/root
set(MATLAB_ROOT C:/Program\ Files/MATLAB/R2024b CACHE PATH "")

# Additional variables that are defined conditionally.
if("${CMAKE_CURRENT_BINARY_DIR}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
    set(BINARY_START_DIR "${START_DIR}")
else()
    set(BINARY_START_DIR "${CMAKE_BINARY_DIR}")
endif()
if(APPLE)
    if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL x86_64)
        list(APPEND MATLAB_ROOT_SYSLIB_PATHS ${MATLAB_ROOT}/bin/maci64)
        list(APPEND MATLAB_ROOT_SYSLIB_PATHS ${MATLAB_ROOT}/sys/os/maci64)
        list(APPEND MATLAB_ROOT_SYSINCLUDE_PATHS ${MATLAB_ROOT}/toolbox/eml/externalDependency/omp/maci64/include)
    elseif("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL arm64)
        list(APPEND MATLAB_ROOT_SYSLIB_PATHS ${MATLAB_ROOT}/bin/maca64)
        list(APPEND MATLAB_ROOT_SYSINCLUDE_PATHS ${MATLAB_ROOT}/toolbox/eml/externalDependency/omp/maca64/include)
    endif()
elseif(UNIX AND ("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL x86_64) AND ("${CMAKE_SYSTEM_NAME}" STREQUAL Linux))
    list(APPEND MATLAB_ROOT_SYSLIB_PATHS ${MATLAB_ROOT}/bin/glnxa64)
elseif(WIN32 AND ("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL AMD64))
    if(MSVC)
        list(APPEND MATLAB_ROOT_SYSLIB_PATHS ${MATLAB_ROOT}/extern/lib/win64/microsoft)
    elseif(MINGW)
        list(APPEND MATLAB_ROOT_SYSLIB_PATHS ${MATLAB_ROOT}/extern/lib/win64/mingw64)
    endif()
    list(APPEND MATLAB_ROOT_SYSLIB_PATHS ${MATLAB_ROOT}/bin/win64
        ${MATLAB_ROOT}/lib/win64)
endif()


if(DEFINED MATLAB_ROOT_SYSLIB_PATHS)
    list(APPEND CMAKE_LIBRARY_PATH ${MATLAB_ROOT_SYSLIB_PATHS})
    link_directories(${MATLAB_ROOT_SYSLIB_PATHS})
endif()
if(DEFINED MATLAB_ROOT_SYSINCLUDE_PATHS)
    list(APPEND CMAKE_INCLUDE_PATH ${MATLAB_ROOT_SYSINCLUDE_PATHS})
endif()



###########################################################################
## Required packages
###########################################################################
find_package(OpenMP REQUIRED)

###########################################################################
## System Libraries
###########################################################################
find_library(FOUND_LIBM m NO_SYSTEM_ENVIRONMENT_PATH PATHS ${CMAKE_C_IMPLICIT_LINK_DIRECTORIES} ${CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES})

###########################################################################
## Target definition and commands
###########################################################################

# Definition of target "run_tag_test_chipotle".
add_library(run_tag_test_chipotle STATIC ${START_DIR}/codegen/lib/run_tag_test_chipotle/coder_fileops.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/coder_platform.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/run_tag_test_chipotle_data.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/rt_nonfinite.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/rtGetNaN.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/rtGetInf.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/run_tag_test_chipotle_initialize.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/run_tag_test_chipotle_terminate.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/run_tag_test_chipotle.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/pwd1.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/strcat.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/fullfile.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/hasIRIPrefix.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/fileManager.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/fread.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/coderFread.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/ifWhileCond.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/mrdivide_helper.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/mtimes.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/minOrMax.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/NoveldaChipParams.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/string1.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/NoveldaDDC.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/linspace.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/mean.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/conv.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/fft.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/FFTImplementationCallback.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/abs.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/findpeaks.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/eml_setop.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/norm.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/colon.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/strcmp.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/sin.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/cos.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/smoothdata.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/circshift.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/round.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/proc_frames.c
    ${START_DIR}/codegen/lib/run_tag_test_chipotle/run_tag_test_chipotle_emxutil.c)

# Set properties for target "run_tag_test_chipotle".
set_target_properties(run_tag_test_chipotle PROPERTIES PREFIX ""
    POSITION_INDEPENDENT_CODE ON
    RUNTIME_OUTPUT_DIRECTORY "${BINARY_START_DIR}/codegen/lib/run_tag_test_chipotle/$<0:>"
    LIBRARY_OUTPUT_DIRECTORY "${BINARY_START_DIR}/codegen/lib/run_tag_test_chipotle/$<0:>"
    ARCHIVE_OUTPUT_DIRECTORY "${BINARY_START_DIR}/codegen/lib/run_tag_test_chipotle/$<0:>")

# Specify language features required for target "run_tag_test_chipotle".
target_compile_features(run_tag_test_chipotle PUBLIC c_std_99)

# Specify compiler preprocessor definitions for target 
# "run_tag_test_chipotle".
target_compile_definitions(run_tag_test_chipotle PRIVATE -DMODEL=run_tag_test_chipotle)

# Specify include directories for target "run_tag_test_chipotle".
target_include_directories(run_tag_test_chipotle PUBLIC $<BUILD_INTERFACE:${START_DIR}/codegen/lib/run_tag_test_chipotle>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/codegen/lib/run_tag_test_chipotle>
    $<BUILD_INTERFACE:${START_DIR}>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>>
    $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/include>)

# Specify library link dependencies for target "run_tag_test_chipotle". 
# CMake generator expressions are used to create a CMakeLists.txt file 
# that supports multiple platforms with differently named system library 
# dependencies.
target_link_libraries(run_tag_test_chipotle PUBLIC $<$<BOOL:${FOUND_LIBM}>:m>
    OpenMP::OpenMP_C
    OpenMP::OpenMP_CXX)


###########################################################################
## Build success message
###########################################################################
add_custom_command(TARGET run_tag_test_chipotle POST_BUILD
     COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --cyan "\\#\\#\\# Created STATIC library: $<TARGET_FILE:run_tag_test_chipotle>")

###########################################################################
## Call toolchain hook function if defined
###########################################################################
if(COMMAND toolchain_target_hook)
    toolchain_target_hook(run_tag_test_chipotle)
endif()
