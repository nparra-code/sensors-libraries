idf_component_register(SRCS "vl6180x_api.c" "vl6180x_i2c.c" "i2c_platform.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES driver)
