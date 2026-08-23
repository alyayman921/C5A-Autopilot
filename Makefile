CXX=g++
UNAME_S := $(shell uname -s)

CXX_SOURCES=\
flightsim.cpp\
$(wildcard src/*)

CXX_DEFS=\
-DUSE_SERIAL\
-DUSE_XLSX

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
-I/usr/include/eigen3\

    ifeq ($(BUILD_TYPE),release)
        CXXFLAGS=-std=c++17
        LDFLAGS=-L/usr/local/lib
        CXX_LIBS=-Wl,-Bstatic -lxlsxio_read -lserial -Wl,-Bdynamic -lexpat -lminizip -lz -lm -lpthread
    else ifeq ($(BUILD_TYPE),debug)
        CXXFLAGS=-std=c++23 -g
        CXX_LIBS=-lm -lxlsxio_read -lserial -lpthread
    else
        CXXFLAGS=-std=c++23
        CXX_LIBS=-lm -lxlsxio_read -lserial -lpthread
    endif
endif

# ---------------- Windows ----------------
ifneq (,$(findstring MINGW,$(UNAME_S)))
    TARGET=Release/Release_Windows_x64/FlightSimulator
    CXX_INCLUDES=\
-Iinc/\
-I/ucrt64/include/eigen3\

    LDFLAGS=-L/ucrt64/lib

    ifeq ($(BUILD_TYPE),release)
        CXXFLAGS=-std=c++23 -static -static-libgcc -static-libstdc++ -O2 -march=native
    else ifeq ($(BUILD_TYPE),debug)
        CXXFLAGS=-std=c++23 -static -static-libgcc -static-libstdc++ -O0 -g
    else
        CXXFLAGS=-std=c++23 -static -static-libgcc -static-libstdc++
    endif

    CXX_LIBS=-lxlsxio_read -lminizip -lz -lbz2 -lexpat -lwinpthread
endif

all: $(TARGET)

debug: clean $(TARGET)

release: clean $(TARGET)

$(TARGET): $(CXX_SOURCES)
	$(CXX) $(CXX_SOURCES) $(CXXFLAGS) $(CXX_INCLUDES) $(LDFLAGS) $(CXX_DEFS) -o $@ $(CXX_LIBS)

clean:
	rm -f $(TARGET)
install:
	mkdir -p /usr/share/3lymnFlightSimulator && cp -r meta /usr/share/3lymnFlightSimulator
	cp $(TARGET) /usr/bin
