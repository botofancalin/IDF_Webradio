set(COMPONENT_ADD_INCLUDEDIRS include)

# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES nghttp )
set(COMPONENT_PRIV_REQUIRES freertos esp32 lwip url_parser http audio_player nvs_flash)

set(COMPONENT_SRCS ./http.c)

register_component()
component_compile_options(-Wno-error=unused-label -Wno-error=return-type -Wno-error=missing-braces -Wno-error=pointer-sign -Wno-error=parentheses -I./include)
