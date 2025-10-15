/*
 * omni_radtan_camera.h
 *
 * Omnidirectional camera with unified projection model and radtan distortion.
 */

#ifndef VIKIT_OMNI_RADTAN_CAMERA_H_
#define VIKIT_OMNI_RADTAN_CAMERA_H_

#include <cmath>
#include <Eigen/Core>
#include <vikit/abstract_camera.h>

namespace vk {

class OmniRadtanCamera : public AbstractCamera {
private:
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double xi_;
  double k1_;
  double k2_;
  double p1_;
  double p2_;
  double k3_;
  bool has_distortion_;

  Eigen::Vector2d undistortRadtan(const Eigen::Vector2d &distorted) const;
  void distortRadtan(const double x, const double y, double &xd, double &yd) const;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OmniRadtanCamera(int width, int height, double scale,
                   double xi, double fx, double fy, double cx, double cy,
                   double k1, double k2, double p1, double p2, double k3);

  virtual ~OmniRadtanCamera() {}

  virtual Eigen::Vector3d cam2world(const double &u, const double &v) const override;
  virtual Eigen::Vector3d cam2world(const Eigen::Vector2d &px) const override;
  virtual Eigen::Vector2d world2cam(const Eigen::Vector3d &xyz_c) const override;
  virtual Eigen::Vector2d world2cam(const Eigen::Vector2d &uv) const override;

  virtual double errorMultiplier2() const override { return std::fabs(0.5 * (fx_ + fy_)); }
  virtual double errorMultiplier() const override { return std::fabs(4.0 * fx_ * fy_); }

  virtual double fx() const override { return fx_; }
  virtual double fy() const override { return fy_; }
  virtual double cx() const override { return cx_; }
  virtual double cy() const override { return cy_; }

  virtual void projectionJacobian(const Eigen::Vector3d &xyz_c,
                                  Eigen::Matrix<double, 2, 3> &J) const override;
};

} // namespace vk

#endif // VIKIT_OMNI_RADTAN_CAMERA_H_
