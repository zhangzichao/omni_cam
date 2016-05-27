#pragma once

#include <memory>
#include <string>
#include <iostream>

#include <Eigen/Dense>

namespace omni_cam {

class OCam;
typedef std::shared_ptr<OCam> OCamPtr;

class OCam
{
 public:
  static constexpr int kInversePolynomialOrder = 12;

  OCam(const Eigen::Matrix<double, 5, 1>& polynomial,
                 const Eigen::Vector2d& principal_point,
                 const Eigen::Vector3d& distortion,
                 const Eigen::Matrix<double, kInversePolynomialOrder, 1>&
                 inverse_polynomial);

  void backProject3(
      const Eigen::Ref<const Eigen::Vector2d>& keypoint,
      Eigen::Vector3d* out_bearing_vector) const;

  void project3(
      const Eigen::Ref<const Eigen::Vector3d>& point_3d,
      Eigen::Vector2d* out_keypoint,
      Eigen::Matrix<double, 2, 3>* out_jacobian_point) const;

  void print(std::ostream& out) const;

  static OCamPtr loadOCam(const std::string& parameter_file);

  // Returns the parameters in Vector form:
  // [polynomial(0) ... polynomial(4), cx_, cy_]
  Eigen::VectorXd getIntrinsicParameters() const;

  // Returns the distortion parameters
  Eigen::VectorXd getDistortionParameters() const;

 private:
  const Eigen::Matrix<double, 5, 1> polynomial_;
  const Eigen::Vector2d principal_point_;
  const Eigen::Matrix<double, 12, 1> inverse_polynomial_;

  const Eigen::Matrix2d affine_correction_;
  const Eigen::Matrix2d affine_correction_inverse_;
};


}  // namespace ocam
