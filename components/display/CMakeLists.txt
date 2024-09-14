set(srcs ".")
set(inc ".")

idf_component_register(SRC_DIRS  "${srcs}"
                       INCLUDE_DIRS "${inc}"
                       REQUIRES lcd_gc9a01)