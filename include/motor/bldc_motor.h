#ifndef MOTOR_DYNAMICS_BLDC_MOTOR_H
#define MOTOR_DYNAMICS_BLDC_MOTOR_H

#include <stdint.h>

#include "motor/dc_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double phase_resistance_ohm;
    double phase_inductance_h;
    double back_emf_v_per_rad_s;
    double torque_constant_nm_per_a;
    uint32_t pole_pairs;
    double rotor_inertia_kg_m2;
    double viscous_friction_nm_per_rad_s;
} md_bldc_config_t;

typedef struct {
    double phase_current_a[3];
    double mechanical_speed_rad_s;
    double mechanical_angle_rad;
} md_bldc_state_t;

typedef struct {
    /* Phase terminal potentials relative to the inverter negative rail. */
    double phase_voltage_v[3];
    double load_torque_nm;
} md_bldc_input_t;

typedef struct {
    double phase_current_a_per_s[3];
    double mechanical_acceleration_rad_s2;
    double mechanical_speed_rad_s;
} md_bldc_derivative_t;

typedef struct {
    double electrical_angle_rad;
    double back_emf_v[3];
    double electromagnetic_torque_nm;
    double electrical_power_w;
    double electromagnetic_power_w;
    double copper_loss_w;
    double viscous_loss_w;
} md_bldc_observation_t;

/* Normalized trapezoidal phase waveform in [-1, 1]. */
double md_bldc_trapezoidal_waveform(double electrical_angle_rad);

md_status_t md_bldc_config_validate(const md_bldc_config_t *config);
md_status_t md_bldc_state_validate(const md_bldc_state_t *state);

md_status_t md_bldc_evaluate_checked(
    const md_bldc_config_t *config,
    const md_bldc_state_t *state,
    const md_bldc_input_t *input,
    md_bldc_derivative_t *derivative);

md_status_t md_bldc_step_rk4_checked(
    const md_bldc_config_t *config,
    md_bldc_state_t *state,
    const md_bldc_input_t *input,
    double dt_s);

md_status_t md_bldc_observe(
    const md_bldc_config_t *config,
    const md_bldc_state_t *state,
    const md_bldc_input_t *input,
    md_bldc_observation_t *observation);

/*
 * Generate an averaged six-step terminal-voltage vector. Sector order:
 * A+B-, A+C-, B+C-, B+A-, C+A-, C+B-.
 */
md_status_t md_bldc_six_step_voltage(
    double dc_bus_voltage_v,
    double duty_cycle,
    uint32_t sector,
    double load_torque_nm,
    md_bldc_input_t *input);

#ifdef __cplusplus
}
#endif

#endif

