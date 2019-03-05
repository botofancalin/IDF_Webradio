set(COMPONENT_ADD_INCLUDEDIRS include)

# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES common audio_renderer)
set(COMPONENT_PRIV_REQUIRES freertos mp3_decoder fifo controls)

set(COMPONENT_SRCS ./audio_player.c)

register_component()
