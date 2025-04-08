file(GLOB SRC_FILES "../EasyProfile/*.c" "../platform/*.c")

idf_component_register(SRCS "main.c" ${SRC_FILES}
                    INCLUDE_DIRS "" "../EasyProfile/include" "../platform/include")
