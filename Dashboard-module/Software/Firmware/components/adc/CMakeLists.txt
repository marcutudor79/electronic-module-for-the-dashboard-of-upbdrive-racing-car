set(srcs "hybrid_selector_task.c")

set(reqs "common"
         "esp_adc")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "inc" REQUIRES "${reqs}")