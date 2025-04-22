idf_component_register(SRCS "vl6180x_lib/vl6180x_lib.c" "vl6180x_api/vl6180x_api.c" "vl6180x_api/vl6180x_i2c.c" "platform/platform_esp32s3.c"
                    INCLUDE_DIRS "." "vl6180x_lib/include" "vl6180x_api/include" "platform/include"
                    PRIV_REQUIRES driver)
