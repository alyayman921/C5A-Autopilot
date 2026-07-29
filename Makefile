UNAME_S := $(shell uname -s)

ifneq (,$(filter release,$(MAKECMDGOALS)))
    BUILD_TYPE = release
    $(info [RELEASE BUILD])
else
    $(info [DEBUG BUILD])
endif

$(info Building $(BUILD_TYPE) mode)
SOURCES = flightsim.cpp

ifeq ($(UNAME_S),Linux)
    CXX = g++
    TARGET = Release/Release_Linux_x64/FlightSimulator
    INCLUDES = -I/usr/include/eigen3
    
    ifeq ($(BUILD_TYPE),release)
    	CXX = g++-9
        CXXFLAGS = -std=c++17 -DUSE_SERIAL
        LDFLAGS = -L/usr/local/lib
        LIBS = -Wl,-Bstatic -lxlsxio_read -lserial -Wl,-Bdynamic -lexpat -lminizip -lz -lm -lpthread
    else
        CXXFLAGS = -std=c++23 -DUSE_SERIAL
        LDFLAGS = -L/usr/local/lib
        LIBS = -lxlsxio_read -lserial -lexpat -lminizip -lz -lm -lpthread
    endif
endif

ifneq (,$(findstring MINGW,$(UNAME_S)))
    CXX = g++
    INCLUDES = -I/ucrt64/include/eigen3
    TARGET = Release/Release_Windows_x64/FlightSimulator
    LDFLAGS = -L/ucrt64/lib
    
    ifeq ($(BUILD_TYPE),release)
        CXXFLAGS = -std=c++23 -static -static-libgcc -static-libstdc++ -O2 -march=native
    else
        CXXFLAGS = -std=c++23 -static -static-libgcc -static-libstdc++ -O0
    endif
    
    LIBS = -lxlsxio_read -lminizip -lz -lbz2 -lexpat -lwinpthread
endif

all: $(TARGET)

debug: BUILD_TYPE = debug
debug: clean $(TARGET)

release: BUILD_TYPE = release
release: clean $(TARGET)

$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LDFLAGS) $< -o $@ $(LIBS)

clean:
	rm -f $(TARGET)
