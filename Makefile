CXX = g++
CXXFLAGS = -std=c++11
SRC = src/sysguage.cpp
TARGET = sysguage

all:
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)

clean:
	rm -f $(TARGET)
