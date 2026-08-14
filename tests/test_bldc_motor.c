#include "motor/bldc_motor.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static md_bldc_config_t fixture_config(void) {
    const md_bldc_config_t config = {
        .phase_resistance_ohm = 0.4,
        .phase_inductance_h = 0.01,
        .back_emf_v_per_rad_s = 0.02,
        .torque_constant_nm_per_a = 0.02,
        .pole_pairs = 7,
        .rotor_inertia_kg_m2 = 0.002,
        .viscous_friction_nm_per_rad_s = 0.001,
    };
    return config;
}

static void assert_close(double actual, double expected, double tolerance) {
    assert(isfinite(actual));
    assert(fabs(actual - expected) <= tolerance);
}

static void test_trapezoidal_waveform(void) {
    assert_close(md_bldc_trapezoidal_waveform(0.0), 0.0, 1e-12);
    assert_close(md_bldc_trapezoidal_waveform(M_PI / 6.0), 1.0, 1e-12);
    assert_close(md_bldc_trapezoidal_waveform(M_PI / 2.0), 1.0, 1e-12);
    assert_close(md_bldc_trapezoidal_waveform(M_PI), 0.0, 1e-12);
    assert_close(md_bldc_trapezoidal_waveform(3.0 * M_PI / 2.0),
                 -1.0, 1e-12);
}

static void test_six_step_locked_rotor_derivative(void) {
    const md_bldc_config_t config = fixture_config();
    const md_bldc_state_t state = {{0.0, 0.0, 0.0}, 0.0, 0.0};
    md_bldc_input_t input;
    md_bldc_derivative_t derivative;

    assert(md_bldc_six_step_voltage(
        12.0, 1.0, 0, 0.0, &input) == MD_STATUS_OK);
    assert(md_bldc_evaluate_checked(
        &config, &state, &input, &derivative) == MD_STATUS_OK);
    assert_close(derivative.phase_current_a_per_s[0], 600.0, 1e-10);
    assert_close(derivative.phase_current_a_per_s[1], -600.0, 1e-10);
    assert_close(derivative.phase_current_a_per_s[2], 0.0, 1e-10);
}

static void test_wye_current_sum_is_preserved(void) {
    const md_bldc_config_t config = fixture_config();
    md_bldc_state_t state = {{0.0, 0.0, 0.0}, 0.0, 0.0};
    md_bldc_input_t input;

    for (size_t step = 0; step < 10000; ++step) {
        const uint32_t sector = (uint32_t)((step / 1000) % 6);
        assert(md_bldc_six_step_voltage(
            12.0, 0.35, sector, 0.01, &input) == MD_STATUS_OK);
        assert(md_bldc_step_rk4_checked(
            &config, &state, &input, 1e-5) == MD_STATUS_OK);
        assert(fabs(state.phase_current_a[0] + state.phase_current_a[1] +
                    state.phase_current_a[2]) < 1e-10);
    }
    assert(isfinite(state.mechanical_speed_rad_s));
    assert(fabs(state.mechanical_speed_rad_s) > 1e-6);
}

static void test_torque_observation(void) {
    const md_bldc_config_t config = fixture_config();
    const md_bldc_state_t state = {{2.0, -2.0, 0.0}, 10.0,
                                   M_PI / (2.0 * 7.0)};
    const md_bldc_input_t input = {{6.0, 0.0, 3.0}, 0.0};
    md_bldc_observation_t observation;

    assert(md_bldc_observe(
        &config, &state, &input, &observation) == MD_STATUS_OK);
    assert_close(observation.electromagnetic_torque_nm, 0.08, 1e-12);
    assert_close(observation.copper_loss_w, 3.2, 1e-12);
}

static void test_failure_preserves_state(void) {
    md_bldc_config_t config = fixture_config();
    const md_bldc_input_t input = {{6.0, 0.0, 3.0}, 0.0};
    md_bldc_state_t state = {{1.0, -1.0, 0.0}, 2.0, 3.0};
    const md_bldc_state_t before = state;

    config.phase_inductance_h = 0.0;
    assert(md_bldc_step_rk4_checked(
        &config, &state, &input, 1e-4) == MD_STATUS_INVALID_CONFIG);
    assert(memcmp(&state, &before, sizeof(state)) == 0);
}

int main(void) {
    test_trapezoidal_waveform();
    test_six_step_locked_rotor_derivative();
    test_wye_current_sum_is_preserved();
    test_torque_observation();
    test_failure_preserves_state();
    puts("motorDynamics BLDC tests passed");
    return 0;
}

