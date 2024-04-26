set(srcs "can_task.c")

set(reqs "common")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "inc" REQUIRES "${reqs}")