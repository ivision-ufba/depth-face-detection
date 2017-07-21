CXX := g++

CXXFLAGS = -Wall -std=c++11 -O3 -ffast-math

INCLUDE = `pkg-config --cflags opencv` -I $(CURDIR)/include

LDFLAGS := `pkg-config --libs opencv`

all: realsense_demo.out realsense_frontal_demo.out frgc_frontal_demo.out

realsense_demo.out: build/realsense_demo.o build/face_detector.o build/realsense.o
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) -lrealsense

build/realsense_demo.o: src/realsense_demo.cpp include/face_detector.hpp include/realsense.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@ $(INCLUDE)

realsense_frontal_demo.out: build/realsense_frontal_demo.o build/face_detector.o build/realsense.o
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) -lrealsense

frgc_frontal_demo.out: build/frgc_frontal_demo.o build/face_detector.o build/abs.o
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

build/realsense_frontal_demo.o: src/realsense_frontal_demo.cpp include/face_detector.hpp include/realsense.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@ $(INCLUDE)

build/frgc_frontal_demo.o: src/frgc_frontal_demo.cpp include/face_detector.hpp include/abs.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@ $(INCLUDE)

build/face_detector.o: src/face_detector.cpp include/face_detector.hpp include/calibration.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@ $(INCLUDE)

build/realsense.o: src/realsense.cpp include/realsense.hpp include/calibration.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@ $(INCLUDE)

build/abs.o: src/abs.cpp include/abs.hpp include/calibration.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@ $(INCLUDE)

clean:
	rm -f build/*.o
	rm -f *.out
