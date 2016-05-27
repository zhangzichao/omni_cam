# omni_cam
C++ implementation of the polynomial omnidirectional camera model by Davide Scaramuzza.
The implementation works with the output of 
the [OCamCalib](https://sites.google.com/site/scarabotix/ocamcalib-toolbox) toolbox.

In this code, the projection and backprojection functions are implemented.
The Jacobian for the projection (i.e., image coordinates w.r.t. 3D point) is implemented as well.

Note that the visibility check in this implementation only checks the boundary of the image. Since images from fisheye and catadioptric cameras usually occupy only part of the image rectangle (e.g., a circle in the center), you probably want to add a mask when using this implementation in your project.

## How to Install
We provide the implementation as a ROS package. The dependencies are:
* [eigen_catkin](https://github.com/ethz-asl/eigen_catkin)
* [glog_catkin](https://github.com/ethz-asl/glog_catkin)
* [eigen_checks](https://github.com/ethz-asl/eigen_checks): only used in unit tests.

Clone the packages into your workspace, then clone this repository and compile.

The first two are just catkin wrappers for `eigen` and `glog`.
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
The output of the calibration toolbox looks like:
```
#polynomial coefficients for the DIRECT mapping function (ocam_model.ss in MATLAB). These are used by cam2world

5 -2.065170e+02 0.000000e+00 2.207161e-03 -4.879622e-06 1.865656e-08 

#polynomial coefficients for the inverse mapping function (ocam_model.invpol in MATLAB). These are used by world2cam

12 294.182260 152.289570 -12.191217 27.959546 8.241525 -1.970397 10.689256 1.871609 -6.437684 0.430037 3.215544 0.921484 

#center: "row" and "column", starting from 0 (C convention)

241.800066 394.552874

#affine parameters "c", "d", "e"

1.000330 0.000219 0.000257

#image size: "height" and "width"

480 752

```
Note that the number of inverse polynomial coefficients (12 in the above case) may vary in your calibration. 

We use 24 parameters for intrinsics. They are defined, by order, as follows
* image width and height (2)
* polynomial coefficients (5)
* image center (2)
* affine distortion (3)
* inverse polynomial coefficients(12)

You need to copy the corresponding parts from the toolbox output and paste them into __one space separated line__ following the above order.
Check the `ocam_param.txt` in the `test` folder for a example.

There are two things that need to be noticed:

1. **image center**: since Matlab uses row major convention, you will need to swap the values before fill them in.
2. **inverse polynomial coefficients**: it is possible that the number of the inverse polynomial coefficients from the toolbox is less than 12. In this case, just pad the polynomial (from the end) with `0.0` to the length of 12.
In theory it can also happen (we rarely encounter it in practice) that the number is more than 12, in this case you will have to modify the code to increase the supported inverse polynomial order.
