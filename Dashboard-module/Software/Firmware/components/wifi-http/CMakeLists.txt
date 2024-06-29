# Sources to be compiled
set(srcs "wifi_task.c"
         "file_server.c")

# Required components from esp-idf for the above sources
set(reqs "common"
         "esp_wifi"
         "esp_http_server"
         "esp-tls"
         "nvs_flash"
         "esp_netif"
         "fatfs"
         "driver"
         "vfs")

set(embed_files "html/index.html")

# Register the tasks in esp-idf build system
idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "inc" REQUIRES "${reqs}"
                       EMBED_TXTFILES "${embed_files}")