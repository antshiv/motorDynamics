#include "motor/six_step_controller.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static void md_six_step_disable(md_six_step_output_t *output) {
    memset(output, 0, sizeof(*output));
    output->phase[0] = MD_PHASE_COAST;
    output->phase[1] = MD_PHASE_COAST;
    output->phase[2] = MD_PHASE_COAST;
}

static int md_six_step_input_is_finite(const md_six_step_input_t *input) {
    return input && isfinite(input->dt_s) &&
        isfinite(input->bus_voltage_v) && isfinite(input->current_a) &&
        isfinite(input->duty_cycle);
}

md_status_t md_six_step_config_validate(const md_six_step_config_t *config) {
    if (!config) {
        return MD_STATUS_NULL_POINTER;
    }
    if (!isfinite(config->min_bus_voltage_v) ||
        !isfinite(config->max_bus_voltage_v) ||
        !isfinite(config->max_current_a) ||
        !isfinite(config->max_duty_cycle) ||
        !isfinite(config->alignment_duty_cycle) ||
        !isfinite(config->alignment_time_s) ||
        !isfinite(config->startup_initial_period_s) ||
        !isfinite(config->startup_final_period_s) ||
        !isfinite(config->commutation_timeout_s) ||
        config->min_bus_voltage_v <= 0.0f ||
        config->max_bus_voltage_v <= config->min_bus_voltage_v ||
        config->max_current_a <= 0.0f || config->max_duty_cycle <= 0.0f ||
        config->max_duty_cycle > 1.0f ||
        config->alignment_duty_cycle < 0.0f ||
        config->alignment_duty_cycle > config->max_duty_cycle ||
        config->alignment_time_s <= 0.0f ||
        config->startup_initial_period_s <= 0.0f ||
        config->startup_final_period_s <= 0.0f ||
        config->startup_final_period_s > config->startup_initial_period_s ||
        config->startup_commutations == 0 ||
        config->commutation_timeout_s <= 0.0f) {
        return MD_STATUS_INVALID_CONFIG;
    }
    return MD_STATUS_OK;
}

void md_six_step_state_init(md_six_step_state_t *state) {
    if (!state) {
        return;
    }
    memset(state, 0, sizeof(*state));
    state->mode = MD_SIX_STEP_DISARMED;
}

md_status_t md_six_step_pattern(
    uint32_t sector,
    float duty_cycle,
    md_six_step_output_t *output) {
    if (!output) {
        return MD_STATUS_NULL_POINTER;
    }
    md_six_step_disable(output);
    if (sector >= 6 || !isfinite(duty_cycle) || duty_cycle < 0.0f ||
        duty_cycle > 1.0f) {
        return MD_STATUS_INVALID_ARGUMENT;
    }
    if (duty_cycle == 0.0f) {
        return MD_STATUS_OK;
    }

    static const uint32_t high_phase[6] = {0, 0, 1, 1, 2, 2};
    static const uint32_t low_phase[6] = {1, 2, 2, 0, 0, 1};
    output->enabled = true;
    output->sector = sector;
    output->duty_cycle = duty_cycle;
    output->phase[high_phase[sector]] = MD_PHASE_PWM_HIGH;
    output->phase[low_phase[sector]] = MD_PHASE_LOW;
    return MD_STATUS_OK;
}

static void md_six_step_disarm(md_six_step_state_t *state) {
    const uint32_t faults = state->fault_flags;
    md_six_step_state_init(state);
    state->fault_flags = faults;
}

static md_status_t md_six_step_latch_fault(
    md_six_step_state_t *state,
    md_six_step_output_t *output,
    uint32_t fault,
    md_status_t status) {
    state->mode = MD_SIX_STEP_FAULT;
    state->fault_flags |= fault;
    md_six_step_disable(output);
    return status;
}

static float md_six_step_startup_period(
    const md_six_step_config_t *config,
    uint32_t startup_step) {
    if (config->startup_commutations <= 1) {
        return config->startup_final_period_s;
    }
    const float fraction = (float)startup_step /
        (float)(config->startup_commutations - 1u);
    return config->startup_initial_period_s +
        fraction * (config->startup_final_period_s -
                    config->startup_initial_period_s);
}

md_status_t md_six_step_control_step(
    const md_six_step_config_t *config,
    md_six_step_state_t *state,
    const md_six_step_input_t *input,
    md_six_step_output_t *output) {
    if (!state || !input || !output) {
        return MD_STATUS_NULL_POINTER;
    }
    md_six_step_disable(output);
    md_status_t status = md_six_step_config_validate(config);
    if (status != MD_STATUS_OK) {
        return status;
    }
    if (!md_six_step_input_is_finite(input) || input->dt_s <= 0.0f ||
        input->current_a < 0.0f || input->duty_cycle < 0.0f ||
        input->duty_cycle > 1.0f || state->sector >= 6 ||
        state->mode > MD_SIX_STEP_FAULT ||
        !isfinite(state->state_elapsed_s) ||
        !isfinite(state->commutation_elapsed_s)) {
        return md_six_step_latch_fault(
            state, output, MD_SIX_STEP_FAULT_INVALID_INPUT,
            MD_STATUS_INVALID_ARGUMENT);
    }

    if (state->mode == MD_SIX_STEP_FAULT) {
        if (input->clear_fault && !input->arm && !input->hardware_fault &&
            input->bus_voltage_v >= config->min_bus_voltage_v &&
            input->bus_voltage_v <= config->max_bus_voltage_v &&
            input->current_a <= config->max_current_a) {
            md_six_step_state_init(state);
        }
        return MD_STATUS_OK;
    }
    if (!input->arm) {
        md_six_step_disarm(state);
        return MD_STATUS_OK;
    }
    if (input->hardware_fault) {
        return md_six_step_latch_fault(
            state, output, MD_SIX_STEP_FAULT_HARDWARE, MD_STATUS_OK);
    }
    if (input->bus_voltage_v < config->min_bus_voltage_v) {
        return md_six_step_latch_fault(
            state, output, MD_SIX_STEP_FAULT_UNDERVOLTAGE, MD_STATUS_OK);
    }
    if (input->bus_voltage_v > config->max_bus_voltage_v) {
        return md_six_step_latch_fault(
            state, output, MD_SIX_STEP_FAULT_OVERVOLTAGE, MD_STATUS_OK);
    }
    if (input->current_a > config->max_current_a) {
        return md_six_step_latch_fault(
            state, output, MD_SIX_STEP_FAULT_OVERCURRENT, MD_STATUS_OK);
    }

    state->state_elapsed_s += input->dt_s;
    state->commutation_elapsed_s += input->dt_s;
    if (state->mode == MD_SIX_STEP_DISARMED) {
        state->mode = MD_SIX_STEP_ALIGNMENT;
        state->sector = 0;
        state->startup_step = 0;
        state->state_elapsed_s = 0.0f;
        state->commutation_elapsed_s = 0.0f;
    } else if (state->mode == MD_SIX_STEP_ALIGNMENT &&
               state->state_elapsed_s >= config->alignment_time_s) {
        state->mode = MD_SIX_STEP_STARTUP;
        state->state_elapsed_s = 0.0f;
        state->commutation_elapsed_s = 0.0f;
    } else if (state->mode == MD_SIX_STEP_STARTUP) {
        const float period = md_six_step_startup_period(
            config, state->startup_step);
        if (state->commutation_elapsed_s >= period) {
            state->sector = (state->sector + 1u) % 6u;
            state->commutation_elapsed_s = 0.0f;
            ++state->startup_step;
            if (state->startup_step >= config->startup_commutations) {
                state->mode = MD_SIX_STEP_RUN;
                state->state_elapsed_s = 0.0f;
            }
        }
    } else if (state->mode == MD_SIX_STEP_RUN) {
        if (input->commutation_event) {
            state->sector = (state->sector + 1u) % 6u;
            state->commutation_elapsed_s = 0.0f;
        } else if (state->commutation_elapsed_s >
                   config->commutation_timeout_s) {
            return md_six_step_latch_fault(
                state, output, MD_SIX_STEP_FAULT_COMMUTATION_TIMEOUT,
                MD_STATUS_OK);
        }
    }

    float duty_cycle = input->duty_cycle;
    if (state->mode == MD_SIX_STEP_ALIGNMENT) {
        duty_cycle = config->alignment_duty_cycle;
    } else if (duty_cycle > config->max_duty_cycle) {
        duty_cycle = config->max_duty_cycle;
    }
    return md_six_step_pattern(state->sector, duty_cycle, output);
}
