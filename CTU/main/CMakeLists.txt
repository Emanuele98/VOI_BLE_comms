set(srcs    "main.c"
            "CTU_states.c"
            "ble_central.c"
            "peer.c"
            "sd_card.c"
            "wifi.c" )
set(include_dirs include)

idf_component_register(SRCS           "${srcs}"
                       INCLUDE_DIRS   "${include_dirs}")
