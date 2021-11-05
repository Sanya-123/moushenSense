file(GLOB_RECURSE api_scr "api/core/src/*.c" "api/platform/src/*.c" "VL53L1X-ESP32-V-3.2.3-master/*.c")

set(sources
    "main.c"
    "uart2.c"
    "net.c"
    "../ESP32-VL53L0X-master/vl53l0x.c"
    ${api_scr}
    "vl53l1x.c"
)

set(includes
    "../ESP32-VL53L0X-master/include"
    "api/core/inc/"
    "api/platform/inc/"
    "VL53L1X-ESP32-V-3.2.3-master/"
)

idf_component_register(SRCS ${sources}
                    INCLUDE_DIRS ${includes})
