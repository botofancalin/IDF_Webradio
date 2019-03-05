set(COMPONENT_ADD_INCLUDEDIRS include)

# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES freertos fifo)

set(COMPONENT_SRCS ./common_buffer.c)

register_component()
