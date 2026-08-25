#ifndef MOTOR_DYNAMICS_SIX_STEP_CONTROLLER_H
#define MOTOR_DYNAMICS_SIX_STEP_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "motor/dc_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MD_PHASE_COAST = 0,
    MD_PHASE_LOW,
    MD_PHASE_PWM_HIGH
} md_phase_drive_t;

typedef enum {
    MD_SIX_STEP_DISARMED = 0,
    MD_SIX_STEP_ALIGNMENT,
    MD_SIX_STEP_STARTUP,
    MD_SIX_STEP_RUN,
    MD_SIX_STEP_FAULT
} md_six_step_mode_t;

typedef enum {
    MD_SIX_STEP_FAULT_NONE = 0,
    MD_SIX_STEP_FAULT_INVALID_INPUT = 1u << 0,
    MD_SIX_STEP_FAULT_UNDERVOLTAGE = 1u << 1,
    MD_SIX_STEP_FAULT_OVERVOLTAGE = 1u << 2,
    MD_SIX_STEP_FAULT_OVERCURRENT = 1u << 3,
    MD_SIX_STEP_FAULT_COMMUTATION_TIMEOUT = 1u << 4,
    MD_SIX_STEP_FAULT_HARDWARE = 1u << 5
} md_six_step_fault_t;

typedef struct {
    float min_bus_voltage_v;
    float max_bus_voltage_v;
    float max_current_a;
    float max_duty_cycle;
    float alignment_duty_cycle;
    float alignment_time_s;
    float startup_initial_period_s;
    float startup_final_period_s;
    uint32_t startup_commutations;
    float commutation_timeout_s;
} md_six_step_config_t;

typedef struct {
    bool arm;
    bool clear_fault;
    bool commutation_event;
    bool hardware_fault;
    float dt_s;
    float bus_voltage_v;
    float current_a;
    float duty_cycle;
} md_six_step_input_t;

typedef struct {
    md_six_step_mode_t mode;
    uint32_t fault_flags;
    uint32_t sector;
    uint32_t startup_step;
    float state_elapsed_s;
    float commutation_elapsed_s;
} md_six_step_state_t;

typedef struct {
    bool enabled;
    uint32_t sector;
    float duty_cycle;
    md_phase_drive_t phase[3];
} md_six_step_output_t;

md_status_t md_six_step_config_validate(const md_six_step_config_t *config);

void md_six_step_state_init(md_six_step_state_t *state);

md_status_t md_six_step_pattern(
    uint32_t sector,
    float duty_cycle,
    md_six_step_output_t *output);

/*
 * Advance the portable control state machine by one sample. A platform adapter
 * owns PWM/ADC/SPI operations and supplies only validated physical feedback.
 * The commutation observer is intentionally external to this first contract.
 */
md_status_t md_six_step_control_step(
    const md_six_step_config_t *config,
    md_six_step_state_t *state,
    const md_six_step_input_t *input,
    md_six_step_output_t *output);

#ifdef __cplusplus
}
#endif

#endif
