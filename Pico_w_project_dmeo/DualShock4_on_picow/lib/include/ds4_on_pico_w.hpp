/**
 * @file ds4_on_pico_w.hpp
 * @version 0.2.0
 * @date 2024-12-31
 *
 * @copyright Copyright (c) 2024 @Akari.
 *
 */
#ifndef PICO_W_FOR_DS4_HPP
#define PICO_W_FOR_DS4_HPP
#include "struct_ds4_on_pico_w.hpp"

#include <string>

class DS4forPicoW {
public:
    struct config {
        std::string mac_address  = "";
        bool blink_led           = true;
        int blink_time_ms_search = 450;
        int blink_time_ms_rescan = 150;
    };

public:
    DS4forPicoW();
    ~DS4forPicoW();
    void setup(config config = (DS4forPicoW::config){ .mac_address = "", .blink_led = true, .blink_time_ms_search = 450, .blink_time_ms_rescan = 150 });
    DualShock4_state get_state();
    char *get_mac_address();
};

#endif
