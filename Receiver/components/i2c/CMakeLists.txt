set(component_src "i2cdev.c")

idf_component_register(SRCS "${component_src}"
                       INCLUDE_DIRS "include")
