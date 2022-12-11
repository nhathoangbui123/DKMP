set(component_src "dht11.c")

idf_component_register(SRCS "${component_src}"
                       INCLUDE_DIRS "include")
