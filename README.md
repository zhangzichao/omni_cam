# omni_cam
C++ implementation of the polynomial omnidirectional camera model by Davide Scaramuzza.
The implementation works with the output of 
the [OCamCalib](https://sites.google.com/site/scarabotix/ocamcalib-toolbox) toolbox.

In this code, the projection and backprojection functions are implemented.
The Jacobian for the projection (i.e., image coordinates w.r.t. 3D point) is implemented as well.

Note that the visibility check in this implementation only checks the boundary of the image. Since images from fisheye and catadioptric cameras usually occupy only part of the image rectangle (e.g., a circle in the center), you probably want to add a mask when using this implementation in your project.

If you use this code in academic context, please cite the following paper:

Zichao Zhang, Henri Rebecq, Christian Forster, Davide Scaramuzza: Benefit of Large Field-of-View Cameras for Visual Odometry, IEEE International Conference on Robotics and Automation (ICRA), 2016.

```
@InProceedings{Zhang16icra,
  author = {Zhang, Zichao and Rebecq, Henri and Forster, Christian and Scaramuzza, Davide},
  title = {Benefit of Large Field-of-View Cameras for Visual Odometry},
  booktitle = {IEEE International Conference on Robotics and Automation (ICRA)},
  year = {2016}
}
```

## How to Install
We provide the implementation as a ROS package. The dependencies are:
* [catkin_simple](https://github.com/catkin/catkin_simple)
* [eigen_catkin](https://github.com/ethz-asl/eigen_catkin)
* [glog_catkin](https://github.com/ethz-asl/glog_catkin)
* [eigen_checks](https://github.com/ethz-asl/eigen_checks): only used in unit tests.
* [gflags_catkin](https://github.com/ethz-asl/gflags_catkin)(optional)

Clone the packages into your workspace, then clone this repository and compile.

The second and the third are just catkin wrappers for `eigen` and `glog`.
If you do not use catkin, it should be easy to adapt the code to work with the plain packages.

## Polynomial Camera Model
The camera model uses polynomials to calculate the projection and backprojection functions.
It is able to deal with fisheye cameras and catadioptric cameras within one framework.

In this implementation, we use 5 parameters for the backprojection and 12 parameters for the projection. For more details, please refer to the page of the Matlab toolbox.

## Coordinate System Definition
We use the column major convention for the __image coordinate system__: 
`x` coordinate is the column index and `y` coordinate the row index.
The `x` `y` axes of the __camera coordinate system__ are aligned with the image coordinate system 
, and the `z` axis is defined so that it is a right hand system.

Note that the Matlab toolbox uses a row-major convention, which is different from this implementation.

## How to use the OCam class
We provide a simple function to load parameter:
```
ocam_ = omni_cam::OCam::loadOCam(ocam_param);
```
You will need a calibration file `ocam_param` to create the camera model.
You can find the usage of the class in more details from the unit tests.


## Convert Matlab toolbox output to Calibration File
Use the script `scripts/matlab_to_cpp.py`
```
./matlab_to_cpp.py input_file output_file
```
