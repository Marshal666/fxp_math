if(NOT DEFINED BINARY_DIR OR NOT DEFINED TARGET)
    message(FATAL_ERROR "Compile-rejection test requires BINARY_DIR and TARGET")
endif()
set(config_args)
if(DEFINED CONFIG AND NOT CONFIG STREQUAL "")
    list(APPEND config_args --config "${CONFIG}")
endif()

# First check that ordinary fixed/integer usage builds with this toolchain.
execute_process(COMMAND "${CMAKE_COMMAND}" --build "${BINARY_DIR}" ${config_args}
    --target fxp_compile_accept
    RESULT_VARIABLE accepted OUTPUT_VARIABLE output ERROR_VARIABLE error)
if(NOT accepted STREQUAL "0")
    message(FATAL_ERROR "Compile-rejection test could not build its valid control: ${output}\n${error}")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" --build "${BINARY_DIR}" ${config_args}
    --target "${TARGET}" RESULT_VARIABLE built
    OUTPUT_VARIABLE output ERROR_VARIABLE error)
if(built STREQUAL "0")
    message(FATAL_ERROR "Floating point use unexpectedly compiled: ${TARGET}")
endif()
if(NOT "${output}\n${error}" MATCHES "deleted|no match|invalid|conversion|convert|C2280|C2676|C2679|C2440")
    message(FATAL_ERROR "Build failed for an unexpected reason: ${output}\n${error}")
endif()
message(STATUS "${TARGET}: compiler rejected floating point use as expected")
