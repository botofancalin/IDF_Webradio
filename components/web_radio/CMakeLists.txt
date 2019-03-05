set(COMPONENT_ADD_INCLUDEDIRS include)

# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES audio_player playlist)
set(COMPONENT_PRIV_REQUIRES freertos driver vector http controls audio_player tcpip_adapter)

set(COMPONENT_SRCS ./web_radio.c)

register_component()
