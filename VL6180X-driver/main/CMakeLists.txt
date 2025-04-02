set(srcs "i2c_vl6180x_calibration.c")

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ".")
