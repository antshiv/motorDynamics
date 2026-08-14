#include "motor/dc_motor.h"

#include <stdio.h>

int main(void) {
    const md_dc_motor_config_t config = {
        2.0, 0.5, 0.1, 0.1, 0.02, 0.02
    };
    const md_dc_motor_input_t input = {12.0, 0.2};
    md_dc_motor_state_t state = {0.0, 0.0, 0.0};
    const double dt = 0.0001;
    const size_t step_count = 20000;

    puts("time_s,current_a,angular_speed_rad_s,shaft_angle_rad");
    printf("%.17g,%.17g,%.17g,%.17g\n", 0.0, state.current_a,
           state.angular_speed_rad_s, state.shaft_angle_rad);
    for (size_t step = 1; step <= step_count; ++step) {
        if (md_dc_motor_step_rk4_checked(
                &config, &state, &input, dt) != MD_STATUS_OK) {
            return 1;
        }
        if (step % 100 == 0) {
            printf("%.17g,%.17g,%.17g,%.17g\n", step * dt,
                   state.current_a, state.angular_speed_rad_s,
                   state.shaft_angle_rad);
        }
    }
    return 0;
}

