# Orthogonal projection images for 3D face detection
This is an official implementation for [Orthogonal projection images for 3D face detection](http://www.sciencedirect.com/science/article/pii/S0167865513003693) implemented in C++ and based on OpenCV.

## Introduction
This is a real-time 3D face detector based on boosted cascade classifiers that uses a scale-invariant image representation to improve both efficiency and efficacy of the detection process, named orthogonal projection images.
In this representation, images are not scanned at multiple scales in order to detect faces with different distances in relation to the camera.
It achieves a high degree of pose invariance by detecting frontal faces not only in the camera viewpoint but also in rotated views of the scene.

## Requirements
* g++ compiler with support to C++11;
* OpenCV, from the [repository](https://github.com/opencv/opencv). We tested it up to version 3.2.0;
* [Optional] [librealsense](https://github.com/IntelRealSense/librealsense). Only necessary to run the demonstrations using the Intel® RealSense™ acquisition devices.

## Installing the demonstrations
We only tested it with Ubuntu 16.04, but any other Linux distribution should de fine.
To compile the point cloud demos:
  * `mkdir build && make`

Then, to compile the Intel® RealSense™ demos:
  * `make realsense`
  
Or the .abs compatible (FRGC's chosen format) ones:
  * `make frgc`

## Usage
The point cloud demonstrations assume that the `point_cloud_demo.cpp` (or `point_cloud_frontal_demo.cpp`) will be modified so that a point cloud in format `std::vector<cv::Point3d>` will be read in `points` prior to calling the methods.

### FRGC
FRGC's compatible demos are run by:
* `./frgc_demo.out $path_to_abs_file`
* `./frgc_frontal_demo.out $path_to_abs_file`

The detected face will be shown and marked in a blue rectangle.

### Intel® RealSense™
Intel® RealSense™ demos require only running the executable `./realsense_demo.out` or `./realsense_frontal_demo.out`.
The detected face will be marked by a blue rectangle in 30FPS realtime.

## Citing
If you find the code in this repository useful in your research, please consider citing:
```
@article{PAMPLONASEGUNDO201472,
  title = "Orthogonal projection images for 3D face detection",
  journal = "Pattern Recognition Letters",
  year = "2014",
  author = "M. Pamplona Segundo and L. Silva and O.R.P. Bellon and S. Sarkar",
}
```


## Documentation and Troubleshooting
While the code is still not fully documented, there are comments in each method relevant to the implementation detailing its purpose, parameters and return value. Questions and problems can be reported as issues in this repository. We would be glad to answer them.
