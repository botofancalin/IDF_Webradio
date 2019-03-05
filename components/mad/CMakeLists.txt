set(COMPONENT_ADD_INCLUDEDIRS "." )

# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS ./decoder.c ./frame.c ./huffman.c ./bit.c ./layer3.c ./stream.c ./align.c ./layer12.c ./version.c ./synth_stereo.c ./fixed.c ./timer.c)

register_component()
component_compile_options(-Wno-error=unused-label -Wno-error=return-type -Wno-error=missing-braces -Wno-error=pointer-sign -Wno-error=parentheses -I./include)
