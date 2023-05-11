set(srcs    "main.c"
            "ble_wpt_cru.c"
            "cru_hw.c"
            "lis3dh.c"
            "esp8266_wrapper.c"
            )
set(include_dirs include)

idf_component_register(SRCS           "${srcs}"
                       INCLUDE_DIRS   "${include_dirs}")
