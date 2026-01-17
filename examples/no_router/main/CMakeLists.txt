idf_component_register(SRCS "no_router.c"
                       INCLUDE_DIRS ".")
dependencies:
  idf: ">=5.4"
  espressif/cjson: "^1.7.19"
