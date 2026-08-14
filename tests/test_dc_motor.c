#include "motor/dc_motor.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static md_dc_motor_config_t fixture_config(void) {
    const md_dc_motor_config_t config = {
        .resistance_ohm = 2.0,
        .inductance_h = 0.5,
        .back_emf_v_per_rad_s = 0.1,
        .torque_constant_nm_per_a = 0.1,
        .rotor_inertia_kg_m2 = 0.02,
        .viscous_friction_nm_per_rad_s = 0.02,
    };
    return config;
}

static void assert_close(double actual, double expected, double tolerance) {
    assert(isfinite(actual));
    assert(fabs(actual - expected) <= tolerance);
}

static void test_initial_derivative(void) {
    const md_dc_motor_config_t config = fixture_config();
    const md_dc_motor_state_t state = {0.0, 0.0, 0.0};
    const md_dc_motor_input_t input = {12.0, 0.0};
    md_dc_motor_derivative_t derivative;

    assert(md_dc_motor_evaluate_checked(
        &config, &state, &input, &derivative) == MD_STATUS_OK);
    assert_close(derivative.current_a_per_s, 24.0, 1e-12);
    assert_close(derivative.angular_acceleration_rad_s2, 0.0, 1e-12);
    assert_close(derivative.angular_speed_rad_s, 0.0, 1e-12);
}

static void test_steady_state(void) {
    const md_dc_motor_config_t config = fixture_config();
    const md_dc_motor_input_t input = {12.0, 0.2};
    md_dc_motor_state_t state;
    md_dc_motor_derivative_t derivative;

    assert(md_dc_motor_steady_state(&config, &input, &state) == MD_STATUS_OK);
    assert_close(state.angular_speed_rad_s, 16.0, 1e-12);
    assert_close(state.current_a, 5.2, 1e-12);
    assert(md_dc_motor_evaluate_checked(
        &config, &state, &input, &derivative) == MD_STATUS_OK);
    assert_close(derivative.current_a_per_s, 0.0, 1e-12);
    assert_close(derivative.angular_acceleration_rad_s2, 0.0, 1e-12);
}

static void test_transient_converges(void) {
    const md_dc_motor_config_t config = fixture_config();
    const md_dc_motor_input_t input = {12.0, 0.2};
    md_dc_motor_state_t state = {0.0, 0.0, 0.0};
    md_dc_motor_state_t expected;

    assert(md_dc_motor_steady_state(&config, &input, &expected) == MD_STATUS_OK);
    for (size_t i = 0; i < 200000; ++i) {
        assert(md_dc_motor_step_rk4_checked(
            &config, &state, &input, 0.0001) == MD_STATUS_OK);
    }
    assert_close(state.current_a, expected.current_a, 1e-7);
    assert_close(state.angular_speed_rad_s,
                 expected.angular_speed_rad_s, 1e-6);
}

static void test_power_observation(void) {
    const md_dc_motor_config_t config = fixture_config();
    const md_dc_motor_input_t input = {12.0, 0.2};
    const md_dc_motor_state_t state = {4.0, 40.0, 0.0};
    md_dc_motor_observation_t observation;

    assert(md_dc_motor_observe(
        &config, &state, &input, &observation) == MD_STATUS_OK);
    assert_close(observation.electromagnetic_torque_nm, 0.4, 1e-12);
    assert_close(observation.back_emf_v, 4.0, 1e-12);
    assert_close(observation.electrical_power_w, 48.0, 1e-12);
    assert_close(observation.electromagnetic_power_w, 16.0, 1e-12);
    assert_close(observation.copper_loss_w, 32.0, 1e-12);
    assert_close(observation.viscous_loss_w, 32.0, 1e-12);
}

static void test_fail_closed_contract(void) {
    md_dc_motor_config_t config = fixture_config();
    const md_dc_motor_input_t input = {12.0, 0.0};
    md_dc_motor_state_t state = {1.0, 2.0, 3.0};
    md_dc_motor_derivative_t derivative = {1.0, 1.0, 1.0};

    config.inductance_h = 0.0;
    assert(md_dc_motor_evaluate_checked(
        &config, &state, &input, &derivative) == MD_STATUS_INVALID_CONFIG);
    assert_close(derivative.current_a_per_s, 0.0, 0.0);
    assert_close(derivative.angular_acceleration_rad_s2, 0.0, 0.0);

    const md_dc_motor_state_t before = state;
    assert(md_dc_motor_step_rk4_checked(
        &config, &state, &input, 0.001) == MD_STATUS_INVALID_CONFIG);
    assert_close(state.current_a, before.current_a, 0.0);
    assert_close(state.angular_speed_rad_s, before.angular_speed_rad_s, 0.0);
    assert_close(state.shaft_angle_rad, before.shaft_angle_rad, 0.0);
}

int main(void) {
    test_initial_derivative();
    test_steady_state();
    test_transient_converges();
    test_power_observation();
    test_fail_closed_contract();
    puts("motorDynamics DC motor tests passed");
    return 0;
}
