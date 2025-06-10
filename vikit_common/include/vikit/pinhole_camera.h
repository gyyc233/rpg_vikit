/*
 * pinhole_camera.h
 *
 *  Created on: Jul 24, 2012
 *      Author: cforster
 */

#ifndef PINHOLE_CAMERA_H_
#define PINHOLE_CAMERA_H_

#include <stdlib.h>
#include <string>
#include <Eigen/Eigen>
#include <vikit/abstract_camera.h>
#include <opencv2/opencv.hpp>

namespace vk {

using namespace std;
using namespace Eigen;

class PinholeCamera : public AbstractCamera {

private:
  const double fx_, fy_;
  const double cx_, cy_;
  bool distortion_;             //!< is it pure pinhole model or has it radial distortion?
  double d_[5];                 //!< [k1,k2,p1,p2,k3] distortion parameters, see http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
  cv::Mat cvK_, cvD_;
  cv::Mat undist_map1_; // 输出的X坐标重映射参数
  cv::Mat undist_map2_; // 输出的Y坐标重映射参数
  bool use_optimization_;
  Matrix3d K_;
  Matrix3d K_inv_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PinholeCamera(double width, double height, double scale,
                double fx, double fy, double cx, double cy,
                double d0=0.0, double d1=0.0, double d2=0.0, double d3=0.0, double d4=0.0);

  ~PinholeCamera();

  void
  initUnistortionMap();

  /// @brief 像素坐标转相机归一化平面坐标，若像素进行了去畸变，则会把归一化平面像素进行归一化
  /// @param x 
  /// @param y 
  /// @return 
  virtual Vector3d
  cam2world(const double& x, const double& y) const;

  /// @brief 像素坐标转相机归一化平面坐标，若像素进行了去畸变，则会把归一化平面像素进行归一化
  /// @param px 
  /// @return 
  virtual Vector3d
  cam2world(const Vector2d& px) const;

  /// @brief 相机坐标系坐标转像素坐标,若有畸变则对相机坐标系坐标进行去畸变再转为像素坐标
  /// @param xyz_c 
  /// @return 
  virtual Vector2d
  world2cam(const Vector3d& xyz_c) const;

  /// @brief 相机坐标系坐标转像素坐标,若有畸变则对相机坐标系坐标进行去畸变再转为像素坐标
  /// @param uv 
  /// @return 
  virtual Vector2d
  world2cam(const Vector2d& uv) const;

  const Vector2d focal_length() const
  {
    return Vector2d(fx_, fy_);
  }

  virtual double errorMultiplier2() const
  {
    return fabs(fx_);
  }

  virtual double errorMultiplier() const
  {
    return fabs(4.0*fx_*fy_);
  }

  inline const Matrix3d& K() const { return K_; };
  inline const Matrix3d& K_inv() const { return K_inv_; };
  virtual double fx() const { return fx_; };
  virtual double fy() const { return fy_; };
  virtual double cx() const { return cx_; };
  virtual double cy() const { return cy_; };
  inline double d0() const { return d_[0]; };
  inline double d1() const { return d_[1]; };
  inline double d2() const { return d_[2]; };
  inline double d3() const { return d_[3]; };
  inline double d4() const { return d_[4]; };

  void undistortImage(const cv::Mat& raw, cv::Mat& rectified);

};

} // end namespace vk


#endif /* PINHOLE_CAMERA_H_ */
