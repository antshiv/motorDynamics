#ifndef MOTOR_DYNAMICS_DC_MOTOR_H
#define MOTOR_DYNAMICS_DC_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MD_STATUS_OK = 0,
    MD_STATUS_NULL_POINTER,
    MD_STATUS_INVALID_ARGUMENT,
    MD_STATUS_INVALID_CONFIG,
    MD_STATUS_NUMERICAL_FAILURE,
    MD_STATUS_SINGULAR_MODEL
} md_status_t;

/* SI units are mandatory throughout this ABI. */
typedef struct {
    double resistance_ohm;
    double inductance_h;
    double back_emf_v_per_rad_s;
    double torque_constant_nm_per_a;
    double rotor_inertia_kg_m2;
    double viscous_friction_nm_per_rad_s;
} md_dc_motor_config_t;

typedef struct {
    double current_a;
    double angular_speed_rad_s;
    double shaft_angle_rad;
} md_dc_motor_state_t;

typedef struct {
    double terminal_voltage_v;
    double load_torque_nm;
} md_dc_motor_input_t;

typedef struct {
    double current_a_per_s;
    double angular_acceleration_rad_s2;
    double angular_speed_rad_s;
} md_dc_motor_derivative_t;

typedef struct {
    double electromagnetic_torque_nm;
    double back_emf_v;
    double electrical_power_w;
    double electromagnetic_power_w;
    double copper_loss_w;
    double viscous_loss_w;
} md_dc_motor_observation_t;

md_status_t md_dc_motor_config_validate(const md_dc_motor_config_t *config);
md_status_t md_dc_motor_state_validate(const md_dc_motor_state_t *state);

/* Evaluate the continuous-time armature and shaft equations. */
md_status_t md_dc_motor_evaluate_checked(
    const md_dc_motor_config_t *config,
    const md_dc_motor_state_t *state,
    const md_dc_motor_input_t *input,
    md_dc_motor_derivative_t *derivative);

/* Advance one fixed RK4 step. State is unchanged if any stage fails. */
md_status_t md_dc_motor_step_rk4_checked(
    const md_dc_motor_config_t *config,
    md_dc_motor_state_t *state,
    const md_dc_motor_input_t *input,
    double dt_s);

/* Solve the constant-input equilibrium for current and shaft speed. */
md_status_t md_dc_motor_steady_state(
    const md_dc_motor_config_t *config,
    const md_dc_motor_input_t *input,
    md_dc_motor_state_t *steady_state);

md_status_t md_dc_motor_observe(
    const md_dc_motor_config_t *config,
    const md_dc_motor_state_t *state,
    const md_dc_motor_input_t *input,
    md_dc_motor_observation_t *observation);

const char *md_status_string(md_status_t status);

#ifdef __cplusplus
}
#endif

#endif
