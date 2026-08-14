#include "motor/bldc_motor.h"

#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main(void) {
    const md_bldc_config_t config = {
        0.4, 0.01, 0.02, 0.02, 7, 0.002, 0.001
    };
    md_bldc_state_t state = {{0.0, 0.0, 0.0}, 0.0, 0.0};
    md_bldc_input_t input = {{0.0, 0.0, 0.0}, 0.01};
    const double dt = 1e-5;
    const size_t step_count = 20000;
    const double excitation_rad_s = 80.0;

    puts("time_s,ia_a,ib_a,ic_a,speed_rad_s,angle_rad");
    printf("0,0,0,0,0,0\n");
    for (size_t step = 1; step <= step_count; ++step) {
        const double time = (double)(step - 1) * dt;
        input.phase_voltage_v[0] = 6.0 + 3.0 * sin(excitation_rad_s * time);
        input.phase_voltage_v[1] = 6.0 + 3.0 * sin(
            excitation_rad_s * time - 2.0 * M_PI / 3.0);
        input.phase_voltage_v[2] = 6.0 + 3.0 * sin(
            excitation_rad_s * time + 2.0 * M_PI / 3.0);
        if (md_bldc_step_rk4_checked(
                &config, &state, &input, dt) != MD_STATUS_OK) {
            return 1;
        }
        if (step % 100 == 0) {
            printf("%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n",
                   step * dt, state.phase_current_a[0],
                   state.phase_current_a[1], state.phase_current_a[2],
                   state.mechanical_speed_rad_s,
                   state.mechanical_angle_rad);
        }
    }
    return 0;
}

