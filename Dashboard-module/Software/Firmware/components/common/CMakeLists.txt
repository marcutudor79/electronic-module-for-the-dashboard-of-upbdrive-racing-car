set(srcs )

set(reqs "driver")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "inc" REQUIRES "${reqs}")