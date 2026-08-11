CXX=g++

CXXFLAGS=\
-std=c++23\
-g\

TARGET=\
Release/Release_Linux_x64/FlightSimulator

CXX_SOURCES=\
flightsim.cpp\
$(wildcard src/*)

CXX_INCLUDES=\
-Iinc/\
-I/usr/include/eigen3\

CXX_LIBS=\
-lm\
-lxlsxio_read\
-lserial\
-lpthread

CXX_DEFS=\
-DUSE_SERIAL

all:	$(TARGET)

$(TARGET):	$(CXX_SOURCES)
	$(CXX) $(CXX_SOURCES) $(CXXFLAGS) $(CXX_INCLUDES) $(CXX_DEFS) $(CXX_LIBS) -o $(TARGET)

clean:
	rm	-f	$(TARGET)
