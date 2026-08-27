CXX=g++
UNAME_S := $(shell uname -s)

CXX_SOURCES=\
flightsim.cpp\
$(wildcard src/*.cpp)\
$(wildcard src/*.c)


# Build type: release (static link, like Makefile.old), debug (-g), or
# normal (default, no debug flags)
ifneq (,$(filter release,$(MAKECMDGOALS)))
    BUILD_TYPE=release
    # $(info [RELEASE BUILD])
else ifneq (,$(filter debug,$(MAKECMDGOALS)))
    BUILD_TYPE=debug
    # $(info [DEBUG BUILD])
else
    BUILD_TYPE=normal
    # $(info [NORMAL BUILD])
endif

# ---------------- Linux ----------------
ifeq ($(UNAME_S),Linux)
    TARGET=Release/Release_Linux_x64/FlightSimulator
    CXX_INCLUDES=\
    -Iinc/\

		CXX_DEFS=\

    ifeq ($(BUILD_TYPE),release)
    		TARGET=Release/Release_Linux_x64/FlightSimulator
        CXXFLAGS=-std=c++17
        LDFLAGS=-L/usr/local/lib
        CXX_LIBS=-Wl,-Bstatic -Wl,-Bdynamic -lm -lpthread
    else ifeq ($(BUILD_TYPE),debug)
        CXXFLAGS=-std=c++23 -g
        CXX_LIBS=-lm -lserial -lpthread
    else
        CXXFLAGS=-std=c++23
        CXX_LIBS=-lm -lserial -lpthread
    endif
endif

# ---------------- Windows ----------------
ifneq (,$(findstring MINGW,$(UNAME_S)))
    TARGET=Release/Release_Windows_x64/FlightSimulator
    CXX_INCLUDES=\
    -Iinc/\

    LDFLAGS=-L/ucrt64/lib

    ifeq ($(BUILD_TYPE),release)
        CXXFLAGS=-std=c++23 -static -static-libgcc -static-libstdc++ -O2 -march=native
    else ifeq ($(BUILD_TYPE),debug)
        CXXFLAGS=-std=c++23 -static -static-libgcc -static-libstdc++ -O0 -g
    else
        CXXFLAGS=-std=c++23 -static -static-libgcc -static-libstdc++
    endif

    CXX_LIBS=-lwinpthread
endif

all: $(TARGET)

debug: clean $(TARGET)

release: clean $(TARGET)

stm: CXX_DEFS += -DUSE_SERIAL
stm: clean $(TARGET)

# ---------------- XLSX build (Eigen + xlsxio) ----------------
# Normal `make` uses the hardcoded matrix-library data path and contains no
# Eigen / xlsxio references. This target enables the spreadsheet path.
xlsx: CXX_DEFS += -DUSE_XLSX
xlsx: CXX_INCLUDES += -I/usr/include/eigen3
xlsx: LDFLAGS += -L/usr/local/lib
xlsx: CXX_LIBS = -Wl,-Bstatic -lxlsxio_read -lserial -Wl,-Bdynamic -lexpat -lminizip -lz -lm -lpthread
xlsx: clean $(TARGET)

$(TARGET): $(CXX_SOURCES)
	$(CXX) $(CXX_SOURCES) $(CXXFLAGS) $(CXX_INCLUDES) $(LDFLAGS) $(CXX_DEFS) -o $@ $(CXX_LIBS)
clean:
	rm -f $(TARGET)
install:
	mkdir -p ~/.local/share/3lymnFlightSimulator && cp -r meta ~/.local/share/3lymnFlightSimulator
	cp $(TARGET) ~/.local/bin
