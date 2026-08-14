#include "motor/bldc_motor.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double md_wrap_angle(double angle) {
    double wrapped = fmod(angle, 2.0 * M_PI);
    if (wrapped < 0.0) {
        wrapped += 2.0 * M_PI;
    }
    return wrapped;
}

double md_bldc_trapezoidal_waveform(double electrical_angle_rad) {
    if (!isfinite(electrical_angle_rad)) {
        return NAN;
    }
    const double angle = md_wrap_angle(electrical_angle_rad);
    if (angle < M_PI / 6.0) {
        return 6.0 * angle / M_PI;
    }
    if (angle < 5.0 * M_PI / 6.0) {
        return 1.0;
    }
    if (angle < 7.0 * M_PI / 6.0) {
        return 1.0 - 6.0 * (angle - 5.0 * M_PI / 6.0) / M_PI;
    }
    if (angle < 11.0 * M_PI / 6.0) {
        return -1.0;
    }
    return -1.0 + 6.0 * (angle - 11.0 * M_PI / 6.0) / M_PI;
}

static int md_bldc_input_is_valid(const md_bldc_input_t *input) {
    if (!input || !isfinite(input->load_torque_nm)) {
        return 0;
    }
    for (size_t phase = 0; phase < 3; ++phase) {
        if (!isfinite(input->phase_voltage_v[phase])) {
            return 0;
        }
    }
    return 1;
}

md_status_t md_bldc_config_validate(const md_bldc_config_t *config) {
    if (!config) {
        return MD_STATUS_NULL_POINTER;
    }
    if (!isfinite(config->phase_resistance_ohm) ||
        !isfinite(config->phase_inductance_h) ||
        !isfinite(config->back_emf_v_per_rad_s) ||
        !isfinite(config->torque_constant_nm_per_a) ||
        !isfinite(config->rotor_inertia_kg_m2) ||
        !isfinite(config->viscous_friction_nm_per_rad_s) ||
        config->phase_resistance_ohm <= 0.0 ||
        config->phase_inductance_h <= 0.0 ||
        config->back_emf_v_per_rad_s < 0.0 ||
        config->torque_constant_nm_per_a <= 0.0 ||
        config->pole_pairs == 0 ||
        config->rotor_inertia_kg_m2 <= 0.0 ||
        config->viscous_friction_nm_per_rad_s < 0.0) {
        return MD_STATUS_INVALID_CONFIG;
    }
    return MD_STATUS_OK;
}

md_status_t md_bldc_state_validate(const md_bldc_state_t *state) {
    if (!state) {
        return MD_STATUS_NULL_POINTER;
    }
    for (size_t phase = 0; phase < 3; ++phase) {
        if (!isfinite(state->phase_current_a[phase])) {
            return MD_STATUS_NUMERICAL_FAILURE;
        }
    }
    if (!isfinite(state->mechanical_speed_rad_s) ||
        !isfinite(state->mechanical_angle_rad)) {
        return MD_STATUS_NUMERICAL_FAILURE;
    }
    return MD_STATUS_OK;
}

static void md_bldc_phase_shapes(
    const md_bldc_config_t *config,
    const md_bldc_state_t *state,
    double shapes[3]) {
    const double electrical_angle =
        (double)config->pole_pairs * state->mechanical_angle_rad;
    shapes[0] = md_bldc_trapezoidal_waveform(electrical_angle);
    shapes[1] = md_bldc_trapezoidal_waveform(
        electrical_angle - 2.0 * M_PI / 3.0);
    shapes[2] = md_bldc_trapezoidal_waveform(
        electrical_angle + 2.0 * M_PI / 3.0);
}

md_status_t md_bldc_evaluate_checked(
    const md_bldc_config_t *config,
    const md_bldc_state_t *state,
    const md_bldc_input_t *input,
    md_bldc_derivative_t *derivative) {
    if (!derivative) {
        return MD_STATUS_NULL_POINTER;
    }
    memset(derivative, 0, sizeof(*derivative));
    md_status_t status = md_bldc_config_validate(config);
    if (status != MD_STATUS_OK) {
        return status;
    }
    status = md_bldc_state_validate(state);
    if (status != MD_STATUS_OK) {
        return status;
    }
    if (!input) {
        return MD_STATUS_NULL_POINTER;
    }
    if (!md_bldc_input_is_valid(input)) {
        return MD_STATUS_INVALID_ARGUMENT;
    }

    double shapes[3];
    double back_emf[3];
    md_bldc_phase_shapes(config, state, shapes);
    double neutral_voltage = 0.0;
    for (size_t phase = 0; phase < 3; ++phase) {
        back_emf[phase] = config->back_emf_v_per_rad_s *
            state->mechanical_speed_rad_s * shapes[phase];
        neutral_voltage += input->phase_voltage_v[phase] - back_emf[phase] -
            config->phase_resistance_ohm * state->phase_current_a[phase];
    }
    neutral_voltage /= 3.0;

    double electromagnetic_torque = 0.0;
    for (size_t phase = 0; phase < 3; ++phase) {
        derivative->phase_current_a_per_s[phase] =
            (input->phase_voltage_v[phase] - neutral_voltage -
             config->phase_resistance_ohm * state->phase_current_a[phase] -
             back_emf[phase]) /
            config->phase_inductance_h;
        electromagnetic_torque += config->torque_constant_nm_per_a *
            shapes[phase] * state->phase_current_a[phase];
    }
    derivative->mechanical_acceleration_rad_s2 =
        (electromagnetic_torque - input->load_torque_nm -
         config->viscous_friction_nm_per_rad_s *
             state->mechanical_speed_rad_s) /
        config->rotor_inertia_kg_m2;
    derivative->mechanical_speed_rad_s = state->mechanical_speed_rad_s;

    for (size_t phase = 0; phase < 3; ++phase) {
        if (!isfinite(derivative->phase_current_a_per_s[phase])) {
            memset(derivative, 0, sizeof(*derivative));
            return MD_STATUS_NUMERICAL_FAILURE;
        }
    }
    if (!isfinite(derivative->mechanical_acceleration_rad_s2) ||
        !isfinite(derivative->mechanical_speed_rad_s)) {
        memset(derivative, 0, sizeof(*derivative));
        return MD_STATUS_NUMERICAL_FAILURE;
    }
    return MD_STATUS_OK;
}

static void md_bldc_add_scaled(
    md_bldc_state_t *output,
    const md_bldc_state_t *base,
    const md_bldc_derivative_t *derivative,
    double scale) {
    for (size_t phase = 0; phase < 3; ++phase) {
        output->phase_current_a[phase] = base->phase_current_a[phase] +
            scale * derivative->phase_current_a_per_s[phase];
    }
    output->mechanical_speed_rad_s = base->mechanical_speed_rad_s +
        scale * derivative->mechanical_acceleration_rad_s2;
    output->mechanical_angle_rad = base->mechanical_angle_rad +
        scale * derivative->mechanical_speed_rad_s;
}

md_status_t md_bldc_step_rk4_checked(
    const md_bldc_config_t *config,
    md_bldc_state_t *state,
    const md_bldc_input_t *input,
    double dt_s) {
    if (!config || !state || !input) {
        return MD_STATUS_NULL_POINTER;
    }
    if (!isfinite(dt_s) || dt_s <= 0.0) {
        return MD_STATUS_INVALID_ARGUMENT;
    }
    const md_bldc_state_t initial = *state;
    md_bldc_state_t stage;
    md_bldc_state_t candidate;
    md_bldc_derivative_t k1;
    md_bldc_derivative_t k2;
    md_bldc_derivative_t k3;
    md_bldc_derivative_t k4;

    md_status_t status = md_bldc_evaluate_checked(
        config, &initial, input, &k1);
    if (status != MD_STATUS_OK) {
        return status;
    }
    md_bldc_add_scaled(&stage, &initial, &k1, 0.5 * dt_s);
    status = md_bldc_evaluate_checked(config, &stage, input, &k2);
    if (status != MD_STATUS_OK) {
        return status;
    }
    md_bldc_add_scaled(&stage, &initial, &k2, 0.5 * dt_s);
    status = md_bldc_evaluate_checked(config, &stage, input, &k3);
    if (status != MD_STATUS_OK) {
        return status;
    }
    md_bldc_add_scaled(&stage, &initial, &k3, dt_s);
    status = md_bldc_evaluate_checked(config, &stage, input, &k4);
    if (status != MD_STATUS_OK) {
        return status;
    }

    for (size_t phase = 0; phase < 3; ++phase) {
        candidate.phase_current_a[phase] = initial.phase_current_a[phase] +
            (dt_s / 6.0) *
            (k1.phase_current_a_per_s[phase] +
             2.0 * k2.phase_current_a_per_s[phase] +
             2.0 * k3.phase_current_a_per_s[phase] +
             k4.phase_current_a_per_s[phase]);
    }
    candidate.mechanical_speed_rad_s = initial.mechanical_speed_rad_s +
        (dt_s / 6.0) *
        (k1.mechanical_acceleration_rad_s2 +
         2.0 * k2.mechanical_acceleration_rad_s2 +
         2.0 * k3.mechanical_acceleration_rad_s2 +
         k4.mechanical_acceleration_rad_s2);
    candidate.mechanical_angle_rad = initial.mechanical_angle_rad +
        (dt_s / 6.0) *
        (k1.mechanical_speed_rad_s + 2.0 * k2.mechanical_speed_rad_s +
         2.0 * k3.mechanical_speed_rad_s + k4.mechanical_speed_rad_s);
    status = md_bldc_state_validate(&candidate);
    if (status != MD_STATUS_OK) {
        return status;
    }
    *state = candidate;
    return MD_STATUS_OK;
}

md_status_t md_bldc_observe(
    const md_bldc_config_t *config,
    const md_bldc_state_t *state,
    const md_bldc_input_t *input,
    md_bldc_observation_t *observation) {
    if (!observation) {
        return MD_STATUS_NULL_POINTER;
    }
    memset(observation, 0, sizeof(*observation));
    md_bldc_derivative_t derivative;
    md_status_t status = md_bldc_evaluate_checked(
        config, state, input, &derivative);
    if (status != MD_STATUS_OK) {
        return status;
    }

    double shapes[3];
    md_bldc_phase_shapes(config, state, shapes);
    observation->electrical_angle_rad = md_wrap_angle(
        (double)config->pole_pairs * state->mechanical_angle_rad);
    for (size_t phase = 0; phase < 3; ++phase) {
        observation->back_emf_v[phase] = config->back_emf_v_per_rad_s *
            state->mechanical_speed_rad_s * shapes[phase];
        observation->electromagnetic_torque_nm +=
            config->torque_constant_nm_per_a * shapes[phase] *
            state->phase_current_a[phase];
        observation->electrical_power_w +=
            input->phase_voltage_v[phase] * state->phase_current_a[phase];
        observation->copper_loss_w += config->phase_resistance_ohm *
            state->phase_current_a[phase] * state->phase_current_a[phase];
    }
    observation->electromagnetic_power_w =
        observation->electromagnetic_torque_nm *
        state->mechanical_speed_rad_s;
    observation->viscous_loss_w =
        config->viscous_friction_nm_per_rad_s *
        state->mechanical_speed_rad_s * state->mechanical_speed_rad_s;
    return MD_STATUS_OK;
}

md_status_t md_bldc_six_step_voltage(
    double dc_bus_voltage_v,
    double duty_cycle,
    uint32_t sector,
    double load_torque_nm,
    md_bldc_input_t *input) {
    if (!input) {
        return MD_STATUS_NULL_POINTER;
    }
    memset(input, 0, sizeof(*input));
    if (!isfinite(dc_bus_voltage_v) || dc_bus_voltage_v <= 0.0 ||
        !isfinite(duty_cycle) || duty_cycle < 0.0 || duty_cycle > 1.0 ||
        sector >= 6 || !isfinite(load_torque_nm)) {
        return MD_STATUS_INVALID_ARGUMENT;
    }
    static const uint32_t high_phase[6] = {0, 0, 1, 1, 2, 2};
    static const uint32_t low_phase[6] = {1, 2, 2, 0, 0, 1};
    const double applied_voltage = dc_bus_voltage_v * duty_cycle;
    input->phase_voltage_v[0] = 0.5 * applied_voltage;
    input->phase_voltage_v[1] = 0.5 * applied_voltage;
    input->phase_voltage_v[2] = 0.5 * applied_voltage;
    input->phase_voltage_v[high_phase[sector]] = applied_voltage;
    input->phase_voltage_v[low_phase[sector]] = 0.0;
    input->load_torque_nm = load_torque_nm;
    return MD_STATUS_OK;
}

