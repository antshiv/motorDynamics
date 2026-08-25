#include "motor/six_step_controller.h"

#include <assert.h>
#include <stdio.h>

static md_six_step_config_t fixture_config(void) {
    const md_six_step_config_t config = {
        .min_bus_voltage_v = 8.0,
        .max_bus_voltage_v = 16.8,
        .max_current_a = 20.0,
        .max_duty_cycle = 0.9,
        .alignment_duty_cycle = 0.1,
        .alignment_time_s = 0.01,
        .startup_initial_period_s = 0.004,
        .startup_final_period_s = 0.001,
        .startup_commutations = 6,
        .commutation_timeout_s = 0.02,
    };
    return config;
}

static md_six_step_input_t fixture_input(void) {
    const md_six_step_input_t input = {
        .arm = true,
        .dt_s = 0.001,
        .bus_voltage_v = 12.0,
        .current_a = 1.0,
        .duty_cycle = 0.4,
    };
    return input;
}

static void test_sector_contract(void) {
    static const md_phase_drive_t expected[6][3] = {
        {MD_PHASE_PWM_HIGH, MD_PHASE_LOW, MD_PHASE_COAST},
        {MD_PHASE_PWM_HIGH, MD_PHASE_COAST, MD_PHASE_LOW},
        {MD_PHASE_COAST, MD_PHASE_PWM_HIGH, MD_PHASE_LOW},
        {MD_PHASE_LOW, MD_PHASE_PWM_HIGH, MD_PHASE_COAST},
        {MD_PHASE_LOW, MD_PHASE_COAST, MD_PHASE_PWM_HIGH},
        {MD_PHASE_COAST, MD_PHASE_LOW, MD_PHASE_PWM_HIGH},
    };
    for (uint32_t sector = 0; sector < 6; ++sector) {
        md_six_step_output_t output;
        assert(md_six_step_pattern(sector, 0.5, &output) == MD_STATUS_OK);
        assert(output.enabled);
        for (uint32_t phase = 0; phase < 3; ++phase) {
            assert(output.phase[phase] == expected[sector][phase]);
        }
    }

    md_six_step_output_t output;
    assert(md_six_step_pattern(0, 0.0, &output) == MD_STATUS_OK);
    assert(!output.enabled);
    assert(output.phase[0] == MD_PHASE_COAST);
    assert(output.phase[1] == MD_PHASE_COAST);
    assert(output.phase[2] == MD_PHASE_COAST);
}

static void test_disarmed_output_is_coast(void) {
    const md_six_step_config_t config = fixture_config();
    md_six_step_input_t input = fixture_input();
    md_six_step_state_t state;
    md_six_step_output_t output;
    md_six_step_state_init(&state);
    input.arm = false;
    assert(md_six_step_control_step(
        &config, &state, &input, &output) == MD_STATUS_OK);
    assert(state.mode == MD_SIX_STEP_DISARMED);
    assert(!output.enabled);
    assert(output.phase[0] == MD_PHASE_COAST);
    assert(output.phase[1] == MD_PHASE_COAST);
    assert(output.phase[2] == MD_PHASE_COAST);
}

static void test_alignment_startup_and_run(void) {
    const md_six_step_config_t config = fixture_config();
    md_six_step_input_t input = fixture_input();
    md_six_step_state_t state;
    md_six_step_output_t output;
    md_six_step_state_init(&state);

    assert(md_six_step_control_step(
        &config, &state, &input, &output) == MD_STATUS_OK);
    assert(state.mode == MD_SIX_STEP_ALIGNMENT);
    assert(output.duty_cycle == config.alignment_duty_cycle);
    for (uint32_t step = 0; step < 200 &&
         state.mode != MD_SIX_STEP_RUN; ++step) {
        assert(md_six_step_control_step(
            &config, &state, &input, &output) == MD_STATUS_OK);
    }
    assert(state.mode == MD_SIX_STEP_RUN);
    const uint32_t sector = state.sector;
    input.commutation_event = true;
    assert(md_six_step_control_step(
        &config, &state, &input, &output) == MD_STATUS_OK);
    assert(state.sector == (sector + 1u) % 6u);
}

static void test_fault_latches_and_requires_disarmed_clear(void) {
    const md_six_step_config_t config = fixture_config();
    md_six_step_input_t input = fixture_input();
    md_six_step_state_t state;
    md_six_step_output_t output;
    md_six_step_state_init(&state);
    input.current_a = 21.0;
    assert(md_six_step_control_step(
        &config, &state, &input, &output) == MD_STATUS_OK);
    assert(state.mode == MD_SIX_STEP_FAULT);
    assert((state.fault_flags & MD_SIX_STEP_FAULT_OVERCURRENT) != 0u);
    assert(!output.enabled);

    input.current_a = 0.0;
    input.clear_fault = true;
    assert(md_six_step_control_step(
        &config, &state, &input, &output) == MD_STATUS_OK);
    assert(state.mode == MD_SIX_STEP_FAULT);
    input.arm = false;
    assert(md_six_step_control_step(
        &config, &state, &input, &output) == MD_STATUS_OK);
    assert(state.mode == MD_SIX_STEP_DISARMED);
    assert(state.fault_flags == MD_SIX_STEP_FAULT_NONE);
}

static void test_run_timeout_fails_closed(void) {
    const md_six_step_config_t config = fixture_config();
    md_six_step_input_t input = fixture_input();
    md_six_step_state_t state;
    md_six_step_output_t output;
    md_six_step_state_init(&state);
    state.mode = MD_SIX_STEP_RUN;
    state.sector = 2;
    input.dt_s = 0.021;
    assert(md_six_step_control_step(
        &config, &state, &input, &output) == MD_STATUS_OK);
    assert(state.mode == MD_SIX_STEP_FAULT);
    assert((state.fault_flags &
            MD_SIX_STEP_FAULT_COMMUTATION_TIMEOUT) != 0u);
    assert(!output.enabled);
}

static void test_invalid_state_fails_closed(void) {
    const md_six_step_config_t config = fixture_config();
    const md_six_step_input_t input = fixture_input();
    md_six_step_state_t state;
    md_six_step_output_t output;
    md_six_step_state_init(&state);
    state.mode = (md_six_step_mode_t)99;
    assert(md_six_step_control_step(
        &config, &state, &input, &output) == MD_STATUS_INVALID_ARGUMENT);
    assert(state.mode == MD_SIX_STEP_FAULT);
    assert((state.fault_flags & MD_SIX_STEP_FAULT_INVALID_INPUT) != 0u);
    assert(!output.enabled);
}

int main(void) {
    test_sector_contract();
    test_disarmed_output_is_coast();
    test_alignment_startup_and_run();
    test_fault_latches_and_requires_disarmed_clear();
    test_run_timeout_fails_closed();
    test_invalid_state_fails_closed();
    puts("motorDynamics six-step controller tests passed");
    return 0;
}
