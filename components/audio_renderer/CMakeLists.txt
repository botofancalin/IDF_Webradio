set(COMPONENT_ADD_INCLUDEDIRS include)

# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES driver common)
set(COMPONENT_PRIV_REQUIRES freertos audio_player)

set(COMPONENT_SRCS ./audio_renderer.c)

register_component()
