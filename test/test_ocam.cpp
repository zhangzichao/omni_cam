#include "omni_cam/ocam.h"

#include <string>
#include <iostream>

#include <gtest/gtest.h>
#include <glog/logging.h>
#include <eigen-checks/gtest.h>

namespace {
std::string getBaseName(const std::string& filename)
{
  const std::string separator = "/";
  std::size_t last_separator = filename.find_last_of(separator);
  if(last_separator == std::string::npos)
  {
    return std::string();
  }
  return filename.substr(0, last_separator);
}
} // namespace


class OCamTest: public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    std::string ocam_param = getBaseName(__FILE__) + "/ocam_param.txt";
    ocam_ = omni_cam::OCam::loadOCam(ocam_param);
    CHECK(ocam_) << "Fail to load the camera: " << ocam_param;
  }

  omni_cam::OCamPtr ocam_;
};

TEST_F(OCamTest, Initialization)
{
  ocam_->print(std::cout);
}

TEST_F(OCamTest, Projection)
{
  constexpr double kEpsProjection = 1e-4;
  const Eigen::Vector3d landmark(1.0, 1.0, -1.0);
  const Eigen::Vector2d expected_keypoint(
        4.729118411664447e+02, 3.929118411664447e+02);

  Eigen::Vector2d keypoint;
  ocam_->project3(landmark, &keypoint, nullptr);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(expected_keypoint, keypoint, kEpsProjection));
}

TEST_F(OCamTest, Backprojection)
{
  constexpr double kEpsBackprojection = 1e-5;
  const Eigen::Vector2d keypoint(400.0, 300.0);
  const Eigen::Vector3d expected_bearing(
        0.733011271294813, 0.549758453471110, 0.400574735838165);

  Eigen::Vector3d bearing;
  ocam_->backProject3(keypoint, &bearing);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(expected_bearing, bearing, kEpsBackprojection));
}


int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  return RUN_ALL_TESTS();
}
