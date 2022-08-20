set(srcs    "main.c"
            "ble_wpt_aux_ctu.c"
            "aux_ctu_hw.c"
            )
set(include_dirs include)

idf_component_register(SRCS           "${srcs}"
                       INCLUDE_DIRS   "${include_dirs}")
