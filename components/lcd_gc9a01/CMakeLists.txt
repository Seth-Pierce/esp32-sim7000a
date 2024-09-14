set(srcs ".")
list(APPEND srcs "fonts")

set(inc ".")
list(APPEND inc "fonts")

idf_component_register(SRC_DIRS  "${srcs}"
                       INCLUDE_DIRS "${inc}"
                       REQUIRES driver)
