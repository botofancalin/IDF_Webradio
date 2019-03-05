set(COMPONENT_ADD_INCLUDEDIRS include)

# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES freertos nvs_flash driver vector fifo audio_renderer web_radio)

set(COMPONENT_SRCS ./app_main.c ./wifi.c)

set(COMPONENT_EMBED_FILES playlist.pls)

register_component()
