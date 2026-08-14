cmake_minimum_required(VERSION 3.16)
project(motor_dynamics VERSION 0.1.0 LANGUAGES C)

set(MD_IS_TOPLEVEL OFF)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(MD_IS_TOPLEVEL ON)
endif()

option(MD_BUILD_TESTS "Build motorDynamics tests" ${MD_IS_TOPLEVEL})
option(MD_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

add_library(motor_dynamics STATIC src/dc_motor.c)
add_library(motorDynamics::motor_dynamics ALIAS motor_dynamics)

target_include_directories(motor_dynamics
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(motor_dynamics PUBLIC c_std_11)
target_compile_options(motor_dynamics PRIVATE
    $<$<C_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>
)
target_link_libraries(motor_dynamics PUBLIC m)

if(MD_ENABLE_SANITIZERS AND CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(motor_dynamics PUBLIC
        -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(motor_dynamics PUBLIC -fsanitize=address,undefined)
endif()

if(MD_BUILD_TESTS)
    enable_testing()

    add_executable(motor_dynamics_tests tests/test_dc_motor.c)
    target_link_libraries(motor_dynamics_tests PRIVATE
        motorDynamics::motor_dynamics)
    target_compile_options(motor_dynamics_tests PRIVATE
        $<$<C_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>
    )
    add_test(NAME motor_dynamics_tests COMMAND motor_dynamics_tests)

    add_executable(dc_motor_trace tests/reference/dc_motor_trace.c)
    target_link_libraries(dc_motor_trace PRIVATE
        motorDynamics::motor_dynamics)
    target_compile_options(dc_motor_trace PRIVATE
        $<$<C_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>
    )

    find_program(UV_EXECUTABLE uv)
    if(UV_EXECUTABLE)
        add_test(
            NAME scipy_dc_motor_parity
            COMMAND ${UV_EXECUTABLE} run --isolated --no-project
                    --with numpy>=2,<3 --with scipy>=1.14,<2
                    python
                    ${CMAKE_CURRENT_SOURCE_DIR}/tests/reference/test_scipy_dc_motor.py
                    --trace $<TARGET_FILE:dc_motor_trace>
        )
    endif()
endif()
