idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES json esp_http_server nvs_flash fatfs ulp arduino-esp32 HULP arduino-libraries)

if(NOT DEFINED TRANSDUCER)
    message(STATUS "No hardware target specified, using default\nSpecify one with idf.py -DTRANSDUCER=<target>\n<target> can be one of DIY3, DIY4, or DIY5")
    set(TRANSDUCER "DIY3")
endif()

if(TRANSDUCER STREQUAL "DIY3")
    add_compile_definitions(DIY3)
elseif(TRANSDUCER STREQUAL "DIY4")
    add_compile_definitions(DIY4)
elseif(TRANSDUCER STREQUAL "DIY5")
    add_compile_definitions(DIY5)
else()
    message(FATAL_ERROR "Invalid hardware target: ${TRANSDUCER}\nSpecify one with idf.py -DTRANSDUCER=<target>\n<target> can be one of DIY3, DIY4, or DIY5")
endif()

message(STATUS "Building firmware for hardware target: ${TRANSDUCER}")    

if(CONFIG_TRANSDUCER_ULP_TOGGLE_GPIO)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TRANSDUCER_ULP_TOGGLE_GPIO)
endif()