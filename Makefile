CC      = gcc
CXX     = g++
CFLAGS  = -O3 -march=native -std=gnu11 -w -Isrc -Igenerated -Ivendor $(EXTRA_CFLAGS)
CXXFLAGS= -O3 -march=native -std=c++17 -w -Ivendor $(EXTRA_CFLAGS)
LDFLAGS = $(EXTRA_LDFLAGS) -L$(HOME)/opt/lib/emulink
LDLIBS  = -lm -lz -lvulkan -lwayland-client -lX11 -lxcb -ldl -lstdc++ -lpthread
SRC     = src/neogeo.c src/video.c src/rom.c src/config.c \
          src/audio.c src/wl_input.c src/render_vulkan.c src/ctl.c src/netplay.c src/main.c \
          src/xdg-shell-protocol.c
OBJ     = $(SRC:src/%.c=build/%.o) build/z80_wrap.o build/cpu_z80.o build/m68kcpu.o build/m68kops.o build/softfloat.o \
          build/ym2610_wrap.o build/ymfm_opn.o build/ymfm_adpcm.o build/ymfm_ssg.o
BIN     = build/linoragx

all: $(BIN)

generated/shaders_spv.h: shaders/quad.vert shaders/quad.frag
	@mkdir -p generated
	glslc --target-env=vulkan1.0 -mfmt=bin -o generated/quad.vert.spv shaders/quad.vert
	glslc --target-env=vulkan1.0 -mfmt=bin -o generated/quad.frag.spv shaders/quad.frag
	python3 tools/embed_shaders.py generated/quad.vert.spv generated/quad.frag.spv generated/shaders_spv.h

build/m68kcpu.o: vendor/m68kcpu.c | generated/shaders_spv.h
	@mkdir -p build
	$(CC) $(CFLAGS) -Ivendor/softfloat -c vendor/m68kcpu.c -o $@

build/m68kops.o: vendor/m68kops.c
	@mkdir -p build
	$(CC) $(CFLAGS) -Ivendor/softfloat -c vendor/m68kops.c -o $@

build/softfloat.o: vendor/softfloat/softfloat.c
	@mkdir -p build
	$(CC) $(CFLAGS) -Ivendor/softfloat -c vendor/softfloat/softfloat.c -o $@

build/cpu_z80.o: vendor/cpu_z80.cpp
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -c vendor/cpu_z80.cpp -o $@

build/z80_wrap.o: src/z80_wrap.cpp vendor/cpu_z80.cpp
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -Isrc -c src/z80_wrap.cpp -o $@

build/ym2610_wrap.o: src/ym2610_wrap.cpp vendor/ymfm/ymfm_opn.h vendor/ymfm/ymfm_ssg.h
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -Isrc -Ivendor/ymfm -c src/ym2610_wrap.cpp -o $@

build/ymfm_opn.o: vendor/ymfm/ymfm_opn.cpp vendor/ymfm/ymfm_fm.ipp
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -Ivendor/ymfm -c vendor/ymfm/ymfm_opn.cpp -o $@

build/ymfm_adpcm.o: vendor/ymfm/ymfm_adpcm.cpp
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -Ivendor/ymfm -c vendor/ymfm/ymfm_adpcm.cpp -o $@

build/ymfm_ssg.o: vendor/ymfm/ymfm_ssg.cpp
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -Ivendor/ymfm -c vendor/ymfm/ymfm_ssg.cpp -o $@

build/%.o: src/%.c | generated/shaders_spv.h
	@mkdir -p build
	$(CC) $(CFLAGS) -c $< -o $@

$(BIN): $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(OBJ) $(LDLIBS)

clean:
	rm -rf build generated

.PHONY: all clean
