idf_component_register(SRCS "ds3231/ds3231.cpp" 
                            "MS5803_05/MS5803_05.cpp" 
                            # "Base64/src/Base64.cpp"
                            # "ArduinoHttpServer/src/internals/HttpResource.cpp" "ArduinoHttpServer/src/internals/HttpVersion.cpp" "ArduinoHttpServer/src/internals/HttpField.cpp" "ArduinoHttpServer/src/internals/StreamHttpReply.cpp"
                      INCLUDE_DIRS "ds3231" "MS5803_05" #"Base64/src" "ArduinoHttpServer/src"
                      REQUIRES arduino-esp32
                      )