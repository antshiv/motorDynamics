#include "motor/hal/motor_hal.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

md_status_t md_motor_hal_validate(const md_motor_hal_t *hal) {
    if (!hal || !hal->ops) {
        return MD_STATUS_NULL_POINTER;
    }
    if (!hal->ops->initialize || !hal->ops->force_safe ||
        !hal->ops->apply_phase_output || !hal->ops->sample ||
        !hal->ops->clear_faults) {
        return MD_STATUS_INVALID_CONFIG;
    }
    return MD_STATUS_OK;
}

md_status_t md_motor_hal_force_safe(const md_motor_hal_t *hal) {
    const md_status_t status = md_motor_hal_validate(hal);
    if (status != MD_STATUS_OK) {
        return status;
    }
    return hal->ops->force_safe(hal->context);
}

md_status_t md_motor_hal_initialize(const md_motor_hal_t *hal) {
    md_status_t status = md_motor_hal_validate(hal);
    if (status != MD_STATUS_OK) {
        return status;
    }
    status = hal->ops->initialize(hal->context);
    if (status != MD_STATUS_OK) {
        return status;
    }
    return hal->ops->force_safe(hal->context);
}

static md_status_t md_motor_hal_output_validate(
    const md_six_step_output_t *output) {
    if (!output) {
        return MD_STATUS_NULL_POINTER;
    }
    if (!isfinite(output->duty_cycle) || output->duty_cycle < 0.0f ||
        output->duty_cycle > 1.0f || output->sector >= 6) {
        return MD_STATUS_INVALID_ARGUMENT;
    }

    uint32_t high_count = 0;
    uint32_t low_count = 0;
    uint32_t coast_count = 0;
    for (uint32_t phase = 0; phase < 3; ++phase) {
        if (output->phase[phase] == MD_PHASE_PWM_HIGH) {
            ++high_count;
        } else if (output->phase[phase] == MD_PHASE_LOW) {
            ++low_count;
        } else if (output->phase[phase] == MD_PHASE_COAST) {
            ++coast_count;
        } else {
            return MD_STATUS_INVALID_ARGUMENT;
        }
    }
    if (output->enabled) {
        if (output->duty_cycle <= 0.0f || high_count != 1 ||
            low_count != 1 || coast_count != 1) {
            return MD_STATUS_INVALID_ARGUMENT;
        }
    } else if (output->duty_cycle != 0.0f || high_count != 0 ||
               low_count != 0 || coast_count != 3) {
        return MD_STATUS_INVALID_ARGUMENT;
    }
    return MD_STATUS_OK;
}

md_status_t md_motor_hal_apply(
    const md_motor_hal_t *hal,
    const md_six_step_output_t *output) {
    md_status_t status = md_motor_hal_validate(hal);
    if (status != MD_STATUS_OK) {
        return status;
    }
    status = md_motor_hal_output_validate(output);
    if (status != MD_STATUS_OK) {
        (void)hal->ops->force_safe(hal->context);
        return status;
    }
    if (!output->enabled) {
        return hal->ops->force_safe(hal->context);
    }
    status = hal->ops->apply_phase_output(hal->context, output);
    if (status != MD_STATUS_OK) {
        (void)hal->ops->force_safe(hal->context);
    }
    return status;
}

md_status_t md_motor_hal_sample(
    const md_motor_hal_t *hal,
    md_motor_hal_sample_t *sample) {
    md_status_t status = md_motor_hal_validate(hal);
    if (status != MD_STATUS_OK) {
        return status;
    }
    if (!sample) {
        return MD_STATUS_NULL_POINTER;
    }
    memset(sample, 0, sizeof(*sample));
    status = hal->ops->sample(hal->context, sample);
    if (status != MD_STATUS_OK) {
        memset(sample, 0, sizeof(*sample));
        return status;
    }
    if (!isfinite(sample->bus_voltage_v) || sample->bus_voltage_v < 0.0f ||
        !isfinite(sample->dc_current_a)) {
        memset(sample, 0, sizeof(*sample));
        return MD_STATUS_NUMERICAL_FAILURE;
    }
    for (uint32_t phase = 0; phase < 3; ++phase) {
        if (!isfinite(sample->phase_current_a[phase]) ||
            !isfinite(sample->bemf_voltage_v[phase])) {
            memset(sample, 0, sizeof(*sample));
            return MD_STATUS_NUMERICAL_FAILURE;
        }
    }
    return MD_STATUS_OK;
}

md_status_t md_motor_hal_clear_faults(const md_motor_hal_t *hal) {
    const md_status_t status = md_motor_hal_validate(hal);
    if (status != MD_STATUS_OK) {
        return status;
    }
    return hal->ops->clear_faults(hal->context);
}
