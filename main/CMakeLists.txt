set(srcs "main.c")
set(inc "../components")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${inc}"
                       REQUIRES uart_internal gpio_internal display sim7000a)
