# Sources to be compiled
set(srcs "button/buttons_task.c"
         "led/heartbeat_task.c"
         "led/led_strip_encoder.c"
         "led/led_task.c")

# Inc directories with header files
set(inc "button/inc"
        "led/inc")

# Required components from esp-idf for the above sources
set(reqs "common"
         "driver")

# Register the tasks in esp-idf build system
idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${inc}" REQUIRES "${reqs}")