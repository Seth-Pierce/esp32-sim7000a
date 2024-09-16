set(srcs ".")
set(inc ".")

idf_component_register(SRC_DIRS  "${srcs}"
                       INCLUDE_DIRS "${inc}"
                       REQUIRES driver uart_internal gpio_internal display)