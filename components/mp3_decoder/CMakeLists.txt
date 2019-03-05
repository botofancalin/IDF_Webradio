set(COMPONENT_ADD_INCLUDEDIRS include)

# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES freertos mad driver audio_renderer audio_player fifo common )

set(COMPONENT_SRCS ./mp3_decoder.c)

register_component()
