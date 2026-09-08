file(MAKE_DIRECTORY "${BINARY_DIR}")
file(WRITE "${BINARY_DIR}/CMakeLists.txt" "cmake_minimum_required(VERSION 3.20)\nproject(reject LANGUAGES CXX)\nadd_executable(reject \"${SOURCE_DIR}/tests/compile_fail/${CASE}.cpp\")\ntarget_include_directories(reject PRIVATE \"${SOURCE_DIR}/include\")\ntarget_compile_features(reject PRIVATE cxx_std_17)\n")
execute_process(COMMAND "${CMAKE_COMMAND}" -S "${BINARY_DIR}" -B "${BINARY_DIR}/build"
    -G "${GENERATOR}" "-DCMAKE_CXX_COMPILER=${CXX_COMPILER}"
    RESULT_VARIABLE configured OUTPUT_VARIABLE output ERROR_VARIABLE error)
if(NOT configured EQUAL 0)
    message(FATAL_ERROR "Compile-rejection test could not configure: ${output}\n${error}")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" --build "${BINARY_DIR}/build" RESULT_VARIABLE built
    OUTPUT_VARIABLE output ERROR_VARIABLE error)
if(built EQUAL 0)
    message(FATAL_ERROR "Floating point use unexpectedly compiled: ${CASE}")
endif()
if(NOT "${output}\n${error}" MATCHES "deleted|no match|invalid|cannot|conversion|convert")
    message(FATAL_ERROR "Build failed for an unexpected reason: ${output}\n${error}")
endif()
