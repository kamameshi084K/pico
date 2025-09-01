
#include "ds4_on_pico_w.hpp"
#include "pico/stdlib.h"

#include <algorithm>
#include <stdio.h>

#define LOG_HEADER "[main]"

int main()
{
    DS4forPicoW controller;
    bool loop_contents = true;
    ////////////////////////////////////////////
    // SETUP
    ////////////////////////////////////////////
    stdio_init_all();
    sleep_ms(5000);
    printf("======================\n[SETUP] DS4 on PicoW\n======================\n");
    // controller.setup((DS4forPicoW::config){ .mac_address = "00:00:00:00:00:00" });
    controller.setup();

    while (1) {
        loop_contents = false;
        do {
            DualShock4_state state = controller.get_state();
            if (true == state.linked) {
                loop_contents = true;
                printf("\n%s [Linked] %s\n", LOG_HEADER, controller.get_mac_address());
                break;
            } else {
                printf(".");
            }
            sleep_ms(250);
        } while (false == loop_contents);

        ////////////////////////////////////////////
        // LOOP
        ////////////////////////////////////////////
        DualShock4_state state;
        printf("%s [LOOP]\n", LOG_HEADER);
        while (loop_contents) {
            tight_loop_contents();
            state = controller.get_state();
            if (true == state.linked) {
                printf("[%01d%s%s%s%s%s%s%s%s%s%s%s] ", //
                       state.hat,
                       state.share ? "S" : " ",
                       state.options ? "O" : " ",
                       state.ps ? "P" : " ",
                       state.cross ? "A" : " ",
                       state.circle ? "B" : " ",
                       state.square ? "X" : " ",
                       state.triangle ? "Y" : " ",
                       state.touch ? "T" : " ",
                       state.mute ? "M" : " ",
                       state.fn1 ? "1" : " ",
                       state.fn2 ? "2" : " ");

                printf("L[%s%s%s(%03d)(%03d,%03d)] ", //
                       state.l1 ? "1" : " ",
                       state.l2 ? "2" : " ",
                       state.l3 ? "3" : " ",
                       state.l2_value,
                       state.l3_x,
                       state.l3_y);
                printf("R[%s%s%s(%03d)(%03d,%03d)] ", //
                       state.r1 ? "1" : " ",
                       state.r2 ? "2" : " ",
                       state.r3 ? "3" : " ",
                       state.r2_value,
                       state.r3_x,
                       state.r3_y);

                printf("Touch[%02d/%3d] ", state.touch_packet_size, state.touch_timestamp);
                printf("T1[%s/%3d(%04d,%03d)] ", state.touch_f1_active ? "x" : " ", state.touch_f1_counter, state.touch_f1_x, state.touch_f1_y);
                printf("T2[%s/%3d(%04d,%03d)] ", state.touch_f2_active ? "x" : " ", state.touch_f2_counter, state.touch_f2_x, state.touch_f2_y);
#if 0
                printf("T1B[%s/%3d(%04d,%03d)] ", state.touch_f1_spare_active ? "x" : " ", state.touch_f1_spare_counter, state.touch_f1_spare_x, state.touch_f1_spare_y);
                printf("T2B[%s/%3d(%04d,%03d)] ", state.touch_f2_spare_active ? "x" : " ", state.touch_f2_spare_counter, state.touch_f2_spare_x, state.touch_f2_spare_y);
#endif
                printf("BtLv.%02d(%03d) ", state.battery_level, state.battery);
                printf("Gyro[%6d,%6d,%6d] ", state.gyro_x, state.gyro_y, state.gyro_z);
                printf("Accel[%6d,%6d,%6d] ", state.accel_x, state.accel_y, state.accel_z);
                printf("%s", state.linked ? " Linked " : "Unlinked");
                printf("in[%s/%s/%s] ", state.connected_usb ? "USB" : "   ", state.connected_mic ? "MIC" : "   ", state.connected_phone ? "Phone" : "     ");
                printf("ID.%02d ", state.report_id);
                printf("TS[%05d] ", state.timestamp);

                printf("\n");
            }
            sleep_ms(20);
        }
        printf("%s [Closing]\n", LOG_HEADER);
    }

    ////////////////////////////////////////////
    // CLOSING
    ////////////////////////////////////////////
    printf("%s [CLOSING] DS4 on PicoW\n", LOG_HEADER);
    stdio_deinit_all();
    return 0;
}
