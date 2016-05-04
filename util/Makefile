OSTYPE = $(shell uname)
ARCH   = $(shell uname -m)
##################################### Defaults ################################
CC             := gcc
CXX            := g++
INCLUDE        := -I.
CFLAGS         := -Wall -O2 -std=c99 -pedantic
CFLAGS_32      := -m32
SHLIB_CFLAGS   := -fPIC -shared
SHLIB_EXT      := .so
LIBS           := -lm
LDFLAGS        :=
############################# Library add-ons #################################
TINYSCHEME_FEATURES := -DUSE_DL=1 -DUSE_MATH=1 -DUSE_ASCII_NAMES=0
INCLUDE += -I/opt/local/include -I/usr/local/include -I./tinyscheme/trunk
LIBS    += -L/opt/local/lib -L/usr/local/lib -lpthread -lhdf5
GLLIBS   =
CFLAGS  += -DH5_NO_DEPRECATED_SYMBOLS
LIBS    += $(GSLLIBS)
############################# OS & ARCH specifics #############################
ifneq ($(OSTYPE), Linux)
  ifeq ($(OSTYPE), Darwin)
    CC = clang
    GLLIBS       += -framework GLUT -framework OpenGL -framework Cocoa
    SHLIB_CFLAGS := -dynamiclib
    SHLIB_EXT    := .dylib
    TINYSCHEME_FEATURES += -DUSE_STRLWR=1 -D__APPLE__=1 -DOSX=1
    ifeq ($(shell sysctl -n hw.optional.x86_64), 1)
      ARCH       := x86_64
    endif
  else ifeq ($(OSTYPE), FreeBSD)
    CC = clang
    GLLIBS += -lGL -lGLU -lglut
  else ifeq ($(OSTYPE), SunOS)
    CFLAGS := -c -Wall -std=c99 -pedantic
  else
    # Let's assume this is win32
    SHLIB_EXT := .dll
    TINYSCHEME_FEATURES += -DUSE_STRLWR=0
  endif
else
  GLLIBS += -lGL -lGLU -lglut
  TINYSCHEME_FEATURES += -DSUN_DL=1
endif

ifneq ($(ARCH), x86_64)
  CFLAGS_32 += -m32
endif

# Are all G5s ppc970s?
ifeq ($(ARCH), ppc970)
  CFLAGS += -m64
endif
############################ Define targets ###################################
EXE_TARGETS = tcpio TMII-8ChipBM waveview TMII-Analog
DEBUG_EXE_TARGETS = hdf5rawWaveformIo
# SHLIB_TARGETS = XXX$(SHLIB_EXT)

ifeq ($(ARCH), x86_64) # compile a 32bit version on 64bit platforms
  # SHLIB_TARGETS += XXX_m32$(SHLIB_EXT)
endif

.PHONY: exe_targets shlib_targets debug_exe_targets clean
exe_targets: $(EXE_TARGETS)
shlib_targets: $(SHLIB_TARGETS)
debug_exe_targets: $(DEBUG_EXE_TARGETS)

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@
tcpio: tcpio.c command.o
	$(CC) $(CFLAGS) $(INCLUDE) $^ $(LIBS) $(LDFLAGS) -o $@
TMII-Analog: TMII-Analog.c command.o
	$(CC) $(CFLAGS) $(INCLUDE) $^ $(LIBS) $(LDFLAGS) -o $@
TMII-1x8: TMII-1x8.c command.o
	$(CC) $(CFLAGS) $(INCLUDE) $^ $(LIBS) $(LDFLAGS) -o $@
waveview: waveview.c hdf5rawWaveformIo.o
	$(CC) $(CFLAGS) $(INCLUDE) -Wno-deprecated-declarations $^ $(LIBS) $(GLLIBS) $(LDFLAGS) -o $@
hdf5rawWaveformIo.o: hdf5rawWaveformIo.c hdf5rawWaveformIo.h common.h
hdf5rawWaveformIo: hdf5rawWaveformIo.c hdf5rawWaveformIo.h
	$(CC) $(CFLAGS) $(INCLUDE) -DHDF5IO_DEBUG_ENABLEMAIN $< $(LIBS) $(LDFLAGS) -o $@
# libmreadarray$(SHLIB_EXT): mreadarray.o
# 	$(CC) $(SHLIB_CFLAGS) $(CFLAGS) $(LIBS) -o $@ $<
# mreadarray.o: mreadarray.c
# 	$(CC) $(CFLAGS) $(INCLUDE) -c -o $@ $<
# mreadarray: mreadarray.c
# 	$(CC) $(CFLAGS) -DENABLEMAIN $(INCLUDE) $(LIBS) -o $@ $<
# libmreadarray_m32$(SHLIB_EXT): mreadarray.c
# 	$(CC) -m32 $(SHLIB_CFLAGS) $(CFLAGS) $(CFLAGS_32) -o $@ $<
LinkDef.h: Makefile
	echo '#pragma link C++ class vector<int>+;' > $@
	echo '#pragma link C++ class vector<long>+;' > $@
	echo '#pragma link C++ class vector<vector<float> >+;' >> $@
	echo '#pragma link C++ class vector<vector<double> >+;' >> $@
Vector.h: Makefile
	echo '#include <vector>' > $@
VectorDict.cpp: Vector.h LinkDef.h
	rootcint -f $@ -c $^
VectorDict.o: VectorDict.cpp VectorDict.h
	$(CXX) $(CFLAGS) $(INCLUDE) $< -c -o $@

clean:
	rm -f *.o *.so *.dylib *.dll *.bundle
	rm -f $(SHLIB_TARGETS) $(EXE_TARGETS) $(DEBUG_EXE_TARGETS)
