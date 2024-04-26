# Sources to be compiled
set(srcs "sdcard/sdcard_task.c")

# Inc directories with header files
set(inc "sdcard/inc")

# Required components from esp-idf for the above sources
set(reqs "common"
         "driver"
         "fatfs")

# Register the tasks in esp-idf build system
idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${inc}" REQUIRES "${reqs}")