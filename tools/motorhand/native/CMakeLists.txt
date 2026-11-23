cmake_minimum_required(VERSION 3.26)
project(ve33_native LANGUAGES CXX)

find_package(Python 3.12 REQUIRED COMPONENTS Interpreter Development.Module Development.SABIModule)
set(MOCHI "${CMAKE_CURRENT_LIST_DIR}/../../../superdex_physics/libraries/mochi")
execute_process(
    COMMAND "${Python_EXECUTABLE}" -c "import superdex.physics as s; print(int(s.USE_DOUBLE_PRECISION))"
    OUTPUT_VARIABLE MOCHI_DOUBLE_PRECISION OUTPUT_STRIP_TRAILING_WHITESPACE
    COMMAND_ERROR_IS_FATAL ANY)
add_subdirectory("${MOCHI}/third_party/nanobind" nanobind)
set(G4 "${CMAKE_CURRENT_LIST_DIR}/../../../hardware/encos_g4_gateway_sdk")
find_package(Threads REQUIRED)
nanobind_add_module(_ve33_native STABLE_ABI NB_DOMAIN mochi NOMINSIZE
    grasp.cpp native_loop.cpp kinematics.cpp "${G4}/shim/g4_shim.cpp")
target_link_libraries(_ve33_native PRIVATE Threads::Threads)
target_compile_features(_ve33_native PRIVATE cxx_std_20)
target_compile_definitions(_ve33_native PRIVATE
    MOCHI_USE_EXTERN_TEMPLATE=0
    MOCHI_USE_DOUBLE_PRECISION=${MOCHI_DOUBLE_PRECISION})
target_include_directories(_ve33_native PRIVATE
    "${G4}/shim" "${G4}/vendor/include"
    "${MOCHI}/mochi_core/include"
    "${MOCHI}/mochi_physics/include"
    "${MOCHI}/third_party/eigen")
