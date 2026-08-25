#include "motor/hal/motor_hal.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

typedef struct {
    uint32_t initialize_calls;
    uint32_t safe_calls;
    uint32_t apply_calls;
    uint32_t sample_calls;
    uint32_t clear_calls;
    md_status_t apply_status;
    bool invalid_sample;
} fake_hal_t;

static md_status_t fake_initialize(void *context) {
    ++((fake_hal_t *)context)->initialize_calls;
    return MD_STATUS_OK;
}

static md_status_t fake_safe(void *context) {
    ++((fake_hal_t *)context)->safe_calls;
    return MD_STATUS_OK;
}

static md_status_t fake_apply(
    void *context,
    const md_six_step_output_t *output) {
    fake_hal_t *fake = context;
    ++fake->apply_calls;
    assert(output->enabled);
    return fake->apply_status;
}

static md_status_t fake_sample(
    void *context,
    md_motor_hal_sample_t *sample) {
    fake_hal_t *fake = context;
    ++fake->sample_calls;
    sample->bus_voltage_v = fake->invalid_sample ? NAN : 12.0f;
    sample->dc_current_a = 1.0f;
    return MD_STATUS_OK;
}

static md_status_t fake_clear(void *context) {
    ++((fake_hal_t *)context)->clear_calls;
    return MD_STATUS_OK;
}

static const md_motor_hal_ops_t fake_ops = {
    .initialize = fake_initialize,
    .force_safe = fake_safe,
    .apply_phase_output = fake_apply,
    .sample = fake_sample,
    .clear_faults = fake_clear,
};

static void test_initialize_forces_safe_output(void) {
    fake_hal_t fake = {0};
    const md_motor_hal_t hal = {.ops = &fake_ops, .context = &fake};
    assert(md_motor_hal_initialize(&hal) == MD_STATUS_OK);
    assert(fake.initialize_calls == 1);
    assert(fake.safe_calls == 1);
}

static void test_valid_pattern_reaches_adapter(void) {
    fake_hal_t fake = {0};
    const md_motor_hal_t hal = {.ops = &fake_ops, .context = &fake};
    md_six_step_output_t output;
    assert(md_six_step_pattern(3, 0.25f, &output) == MD_STATUS_OK);
    assert(md_motor_hal_apply(&hal, &output) == MD_STATUS_OK);
    assert(fake.apply_calls == 1);
    assert(fake.safe_calls == 0);
}

static void test_disabled_pattern_uses_safe_operation(void) {
    fake_hal_t fake = {0};
    const md_motor_hal_t hal = {.ops = &fake_ops, .context = &fake};
    md_six_step_output_t output;
    assert(md_six_step_pattern(0, 0.0f, &output) == MD_STATUS_OK);
    assert(md_motor_hal_apply(&hal, &output) == MD_STATUS_OK);
    assert(fake.apply_calls == 0);
    assert(fake.safe_calls == 1);
}

static void test_invalid_pattern_never_reaches_adapter(void) {
    fake_hal_t fake = {0};
    const md_motor_hal_t hal = {.ops = &fake_ops, .context = &fake};
    md_six_step_output_t output;
    assert(md_six_step_pattern(0, 0.25f, &output) == MD_STATUS_OK);
    output.phase[2] = MD_PHASE_LOW;
    assert(md_motor_hal_apply(&hal, &output) == MD_STATUS_INVALID_ARGUMENT);
    assert(fake.apply_calls == 0);
    assert(fake.safe_calls == 1);
}

static void test_adapter_failure_forces_safe_output(void) {
    fake_hal_t fake = {.apply_status = MD_STATUS_HARDWARE_FAILURE};
    const md_motor_hal_t hal = {.ops = &fake_ops, .context = &fake};
    md_six_step_output_t output;
    assert(md_six_step_pattern(2, 0.5f, &output) == MD_STATUS_OK);
    assert(md_motor_hal_apply(&hal, &output) ==
           MD_STATUS_HARDWARE_FAILURE);
    assert(fake.apply_calls == 1);
    assert(fake.safe_calls == 1);
}

static void test_nonfinite_sample_is_rejected(void) {
    fake_hal_t fake = {0};
    const md_motor_hal_t hal = {.ops = &fake_ops, .context = &fake};
    md_motor_hal_sample_t sample;
    assert(md_motor_hal_sample(&hal, &sample) == MD_STATUS_OK);
    assert(sample.bus_voltage_v == 12.0f);

    fake.invalid_sample = true;
    assert(md_motor_hal_sample(&hal, &sample) ==
           MD_STATUS_NUMERICAL_FAILURE);
    assert(sample.bus_voltage_v == 0.0f);
}

int main(void) {
    test_initialize_forces_safe_output();
    test_valid_pattern_reaches_adapter();
    test_disabled_pattern_uses_safe_operation();
    test_invalid_pattern_never_reaches_adapter();
    test_adapter_failure_forces_safe_output();
    test_nonfinite_sample_is_rejected();
    puts("motorDynamics HAL tests passed");
    return 0;
}
