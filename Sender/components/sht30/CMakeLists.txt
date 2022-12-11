set(component_src "sht30.c" "../i2c/i2cdev.c")

idf_component_register(SRCS "${component_src}"
                       INCLUDE_DIRS "include")
