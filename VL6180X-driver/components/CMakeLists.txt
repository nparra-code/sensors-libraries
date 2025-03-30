idf_component_register(SRCS "vl6180x_api/vl6180x_api.c" "vl6180x_api/vl6180x_i2c.c" "platform/i2c_platform.c"
                    INCLUDE_DIRS "." "vl6180x_api/include" "platform/include"
                    PRIV_REQUIRES driver)
