# Sources to be compiled
set(srcs "lcd-4dsystems/lcd_task.c")

# Inc directories with header files
set(inc "lcd-4dsystems/inc")

# Required components from esp-idf for the above sources
set(reqs "common"
         "driver")

# Register the tasks in esp-idf build system
idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${inc}" REQUIRES "${reqs}")