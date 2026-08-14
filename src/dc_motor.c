#include "motor/dc_motor.h"

#include <math.h>
#include <string.h>

static int md_is_finite_config(const md_dc_motor_config_t *config) {
    return config &&
        isfinite(config->resistance_ohm) &&
        isfinite(config->inductance_h) &&
        isfinite(config->back_emf_v_per_rad_s) &&
        isfinite(config->torque_constant_nm_per_a) &&
        isfinite(config->rotor_inertia_kg_m2) &&
        isfinite(config->viscous_friction_nm_per_rad_s);
}

static int md_is_finite_input(const md_dc_motor_input_t *input) {
    return input && isfinite(input->terminal_voltage_v) &&
        isfinite(input->load_torque_nm);
}

md_status_t md_dc_motor_config_validate(const md_dc_motor_config_t *config) {
    if (!config) {
        return MD_STATUS_NULL_POINTER;
    }
    if (!md_is_finite_config(config) || config->resistance_ohm <= 0.0 ||
        config->inductance_h <= 0.0 ||
        config->back_emf_v_per_rad_s < 0.0 ||
        config->torque_constant_nm_per_a <= 0.0 ||
        config->rotor_inertia_kg_m2 <= 0.0 ||
        config->viscous_friction_nm_per_rad_s < 0.0) {
        return MD_STATUS_INVALID_CONFIG;
    }
    return MD_STATUS_OK;
}

md_status_t md_dc_motor_state_validate(const md_dc_motor_state_t *state) {
    if (!state) {
        return MD_STATUS_NULL_POINTER;
    }
    if (!isfinite(state->current_a) ||
        !isfinite(state->angular_speed_rad_s) ||
        !isfinite(state->shaft_angle_rad)) {
        return MD_STATUS_NUMERICAL_FAILURE;
    }
    return MD_STATUS_OK;
}

md_status_t md_dc_motor_evaluate_checked(
    const md_dc_motor_config_t *config,
    const md_dc_motor_state_t *state,
    const md_dc_motor_input_t *input,
    md_dc_motor_derivative_t *derivative) {
    if (!derivative) {
        return MD_STATUS_NULL_POINTER;
    }
    memset(derivative, 0, sizeof(*derivative));

    md_status_t status = md_dc_motor_config_validate(config);
    if (status != MD_STATUS_OK) {
        return status;
    }
    status = md_dc_motor_state_validate(state);
    if (status != MD_STATUS_OK) {
        return status;
    }
    if (!input) {
        return MD_STATUS_NULL_POINTER;
    }
    if (!md_is_finite_input(input)) {
        return MD_STATUS_INVALID_ARGUMENT;
    }

    derivative->current_a_per_s =
        (input->terminal_voltage_v - config->resistance_ohm * state->current_a -
         config->back_emf_v_per_rad_s * state->angular_speed_rad_s) /
        config->inductance_h;
    derivative->angular_acceleration_rad_s2 =
        (config->torque_constant_nm_per_a * state->current_a -
         config->viscous_friction_nm_per_rad_s *
             state->angular_speed_rad_s -
         input->load_torque_nm) /
        config->rotor_inertia_kg_m2;
    derivative->angular_speed_rad_s = state->angular_speed_rad_s;

    if (!isfinite(derivative->current_a_per_s) ||
        !isfinite(derivative->angular_acceleration_rad_s2) ||
        !isfinite(derivative->angular_speed_rad_s)) {
        memset(derivative, 0, sizeof(*derivative));
        return MD_STATUS_NUMERICAL_FAILURE;
    }
    return MD_STATUS_OK;
}

static void md_state_add_scaled(
    md_dc_motor_state_t *output,
    const md_dc_motor_state_t *base,
    const md_dc_motor_derivative_t *derivative,
    double scale) {
    output->current_a = base->current_a +
        scale * derivative->current_a_per_s;
    output->angular_speed_rad_s = base->angular_speed_rad_s +
        scale * derivative->angular_acceleration_rad_s2;
    output->shaft_angle_rad = base->shaft_angle_rad +
        scale * derivative->angular_speed_rad_s;
}

md_status_t md_dc_motor_step_rk4_checked(
    const md_dc_motor_config_t *config,
    md_dc_motor_state_t *state,
    const md_dc_motor_input_t *input,
    double dt_s) {
    if (!state || !config || !input) {
        return MD_STATUS_NULL_POINTER;
    }
    if (!isfinite(dt_s) || dt_s <= 0.0) {
        return MD_STATUS_INVALID_ARGUMENT;
    }

    const md_dc_motor_state_t initial = *state;
    md_dc_motor_state_t stage;
    md_dc_motor_state_t candidate;
    md_dc_motor_derivative_t k1;
    md_dc_motor_derivative_t k2;
    md_dc_motor_derivative_t k3;
    md_dc_motor_derivative_t k4;

    md_status_t status = md_dc_motor_evaluate_checked(
        config, &initial, input, &k1);
    if (status != MD_STATUS_OK) {
        return status;
    }
    md_state_add_scaled(&stage, &initial, &k1, 0.5 * dt_s);
    status = md_dc_motor_evaluate_checked(config, &stage, input, &k2);
    if (status != MD_STATUS_OK) {
        return status;
    }
    md_state_add_scaled(&stage, &initial, &k2, 0.5 * dt_s);
    status = md_dc_motor_evaluate_checked(config, &stage, input, &k3);
    if (status != MD_STATUS_OK) {
        return status;
    }
    md_state_add_scaled(&stage, &initial, &k3, dt_s);
    status = md_dc_motor_evaluate_checked(config, &stage, input, &k4);
    if (status != MD_STATUS_OK) {
        return status;
    }

    candidate.current_a = initial.current_a + (dt_s / 6.0) *
        (k1.current_a_per_s + 2.0 * k2.current_a_per_s +
         2.0 * k3.current_a_per_s + k4.current_a_per_s);
    candidate.angular_speed_rad_s = initial.angular_speed_rad_s +
        (dt_s / 6.0) *
        (k1.angular_acceleration_rad_s2 +
         2.0 * k2.angular_acceleration_rad_s2 +
         2.0 * k3.angular_acceleration_rad_s2 +
         k4.angular_acceleration_rad_s2);
    candidate.shaft_angle_rad = initial.shaft_angle_rad + (dt_s / 6.0) *
        (k1.angular_speed_rad_s + 2.0 * k2.angular_speed_rad_s +
         2.0 * k3.angular_speed_rad_s + k4.angular_speed_rad_s);

    status = md_dc_motor_state_validate(&candidate);
    if (status != MD_STATUS_OK) {
        return status;
    }
    *state = candidate;
    return MD_STATUS_OK;
}

md_status_t md_dc_motor_steady_state(
    const md_dc_motor_config_t *config,
    const md_dc_motor_input_t *input,
    md_dc_motor_state_t *steady_state) {
    if (!steady_state) {
        return MD_STATUS_NULL_POINTER;
    }
    memset(steady_state, 0, sizeof(*steady_state));
    md_status_t status = md_dc_motor_config_validate(config);
    if (status != MD_STATUS_OK) {
        return status;
    }
    if (!input) {
        return MD_STATUS_NULL_POINTER;
    }
    if (!md_is_finite_input(input)) {
        return MD_STATUS_INVALID_ARGUMENT;
    }

    const double denominator =
        config->torque_constant_nm_per_a *
            config->back_emf_v_per_rad_s +
        config->resistance_ohm *
            config->viscous_friction_nm_per_rad_s;
    if (!isfinite(denominator) || denominator <= 0.0) {
        return MD_STATUS_SINGULAR_MODEL;
    }

    steady_state->angular_speed_rad_s =
        (config->torque_constant_nm_per_a * input->terminal_voltage_v -
         config->resistance_ohm * input->load_torque_nm) /
        denominator;
    steady_state->current_a =
        (input->terminal_voltage_v -
         config->back_emf_v_per_rad_s *
             steady_state->angular_speed_rad_s) /
        config->resistance_ohm;
    return md_dc_motor_state_validate(steady_state);
}

md_status_t md_dc_motor_observe(
    const md_dc_motor_config_t *config,
    const md_dc_motor_state_t *state,
    const md_dc_motor_input_t *input,
    md_dc_motor_observation_t *observation) {
    if (!observation) {
        return MD_STATUS_NULL_POINTER;
    }
    memset(observation, 0, sizeof(*observation));
    md_status_t status = md_dc_motor_config_validate(config);
    if (status != MD_STATUS_OK) {
        return status;
    }
    status = md_dc_motor_state_validate(state);
    if (status != MD_STATUS_OK) {
        return status;
    }
    if (!input) {
        return MD_STATUS_NULL_POINTER;
    }
    if (!md_is_finite_input(input)) {
        return MD_STATUS_INVALID_ARGUMENT;
    }

    observation->electromagnetic_torque_nm =
        config->torque_constant_nm_per_a * state->current_a;
    observation->back_emf_v =
        config->back_emf_v_per_rad_s * state->angular_speed_rad_s;
    observation->electrical_power_w =
        input->terminal_voltage_v * state->current_a;
    observation->shaft_power_w =
        observation->electromagnetic_torque_nm *
        state->angular_speed_rad_s;
    observation->copper_loss_w =
        config->resistance_ohm * state->current_a * state->current_a;

    if (!isfinite(observation->electromagnetic_torque_nm) ||
        !isfinite(observation->back_emf_v) ||
        !isfinite(observation->electrical_power_w) ||
        !isfinite(observation->shaft_power_w) ||
        !isfinite(observation->copper_loss_w)) {
        memset(observation, 0, sizeof(*observation));
        return MD_STATUS_NUMERICAL_FAILURE;
    }
    return MD_STATUS_OK;
}

const char *md_status_string(md_status_t status) {
    switch (status) {
        case MD_STATUS_OK:
            return "ok";
        case MD_STATUS_NULL_POINTER:
            return "null pointer";
        case MD_STATUS_INVALID_ARGUMENT:
            return "invalid argument";
        case MD_STATUS_INVALID_CONFIG:
            return "invalid configuration";
        case MD_STATUS_NUMERICAL_FAILURE:
            return "numerical failure";
        case MD_STATUS_SINGULAR_MODEL:
            return "singular model";
        default:
            return "unknown status";
    }
}

