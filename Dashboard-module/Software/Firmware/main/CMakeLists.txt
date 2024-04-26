# Sources to be compiled
set(srcs "main.c")

# Required components from esp-idf for the above sources
set(reqs "adc"
         "can"
         "common"
         "gpio"
         "i2c"
         "spi"
         "uart"
         "wifi-http")

# Register the tasks in esp-idf build system
idf_component_register(SRCS "${srcs}" REQUIRES "${reqs}")