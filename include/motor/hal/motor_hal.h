#ifndef MOTOR_DYNAMICS_MOTOR_HAL_H
#define MOTOR_DYNAMICS_MOTOR_HAL_H

#include <stdbool.h>
#include <stdint.h>

#include "motor/six_step_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MD_MOTOR_HAL_FAULT_NONE = 0,
    MD_MOTOR_HAL_FAULT_OVERCURRENT = 1u << 0,
    MD_MOTOR_HAL_FAULT_UNDERVOLTAGE = 1u << 1,
    MD_MOTOR_HAL_FAULT_OVERTEMPERATURE = 1u << 2,
    MD_MOTOR_HAL_FAULT_GATE_DRIVER = 1u << 3,
    MD_MOTOR_HAL_FAULT_COMMUNICATION = 1u << 4,
    MD_MOTOR_HAL_FAULT_WATCHDOG = 1u << 5,
    MD_MOTOR_HAL_FAULT_ADC_TIMING = 1u << 6
} md_motor_hal_fault_t;

typedef struct {
    uint32_t timestamp_ticks;
    float bus_voltage_v;
    float dc_current_a;
    float phase_current_a[3];
    float bemf_voltage_v[3];
    uint32_t fault_flags;
    uint32_t predriver_status_raw;
    bool commutation_event;
} md_motor_hal_sample_t;

typedef struct {
    md_status_t (*initialize)(void *context);
    md_status_t (*force_safe)(void *context);
    md_status_t (*apply_phase_output)(
        void *context,
        const md_six_step_output_t *output);
    md_status_t (*sample)(void *context, md_motor_hal_sample_t *sample);
    md_status_t (*clear_faults)(void *context);
} md_motor_hal_ops_t;

typedef struct {
    const md_motor_hal_ops_t *ops;
    void *context;
} md_motor_hal_t;

md_status_t md_motor_hal_validate(const md_motor_hal_t *hal);
md_status_t md_motor_hal_initialize(const md_motor_hal_t *hal);
md_status_t md_motor_hal_force_safe(const md_motor_hal_t *hal);
md_status_t md_motor_hal_apply(
    const md_motor_hal_t *hal,
    const md_six_step_output_t *output);
md_status_t md_motor_hal_sample(
    const md_motor_hal_t *hal,
    md_motor_hal_sample_t *sample);
md_status_t md_motor_hal_clear_faults(const md_motor_hal_t *hal);

#ifdef __cplusplus
}
#endif

#endif
