# set(COMPONENT_REQUIRES "arduino_libraries")

idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES fatfs ulp arduino-esp32 HULP arduino-libraries)
