add_executable(ohashi-v2
    ohashi-v2.cpp
)
target_link_libraries(ohashi-v2 
    pico_stdlib 
    hardware_spi 
    hardware_uart 
    hardware_timer
    ) 

pico_enable_stdio_usb(ohashi-v2 1)
pico_add_extra_outputs(ohashi-v2)
