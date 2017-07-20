CXX := g++

CXXFLAGS = -Wall -std=c++11 -O3 -ffast-math

INCLUDE = `pkg-config --cflags opencv` -I $(CURDIR)/include

LDFLAGS := `pkg-config --libs opencv` -lrealsense

all: detector_demo.out

detector_demo.out: build/main.o build/face_detector.o build/realsense.o
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

build/main.o: src/main.cpp include/face_detector.hpp include/realsense.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@ $(INCLUDE)

build/face_detector.o: src/face_detector.cpp include/face_detector.hpp include/calibration.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@ $(INCLUDE)

build/realsense.o: src/realsense.cpp include/realsense.hpp include/calibration.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@ $(INCLUDE)

clean:
	rm -f build/*.o
	rm -f *.out
