/*
 * abstract_camera.h
 *
 *  Created on: Jul 23, 2012
 *      Author: cforster
 */

#ifndef ABSTRACT_CAMERA_H_
#define ABSTRACT_CAMERA_H_

#include <cmath>
#include <Eigen/Core>

namespace vk
{

using namespace std;
using namespace Eigen;

class AbstractCamera
{
protected:

  int width_;   // TODO cannot be const because of omni-camera model
  int height_;
  double scale_;

public:

  AbstractCamera() {}; // need this constructor for omni camera
  AbstractCamera(int width, int height, double scale) : width_(width), height_(height), scale_(scale){};

  virtual ~AbstractCamera() {};

  /// Project from pixels to world coordiantes. Returns a bearing vector of unit length.
  virtual Vector3d
  cam2world(const double& x, const double& y) const = 0;

  /// Project from pixels to world coordiantes. Returns a bearing vector of unit length.
  virtual Vector3d
  cam2world(const Vector2d& px) const = 0;

  virtual Vector2d
  world2cam(const Vector3d& xyz_c) const = 0;

  /// projects unit plane coordinates to camera coordinates
  virtual Vector2d
  world2cam(const Vector2d& uv) const = 0;

  virtual double
  errorMultiplier2() const = 0;

  virtual double
  errorMultiplier() const = 0;

  virtual double fx() const = 0;
  virtual double fy() const = 0;
  virtual double cx() const = 0;
  virtual double cy() const = 0;

  virtual void projectionJacobian(const Vector3d& xyz_c, Eigen::Matrix<double, 2, 3>& J) const
  {
    const double fx_i = fx();
    const double fy_i = fy();

    const double x = xyz_c[0];
    const double y = xyz_c[1];
    const double z = xyz_c[2];
    if(std::fabs(z) < 1e-12)
    {
      J.setZero();
      return;
    }

    const double z_inv = 1.0 / z;
    const double z_inv2 = z_inv * z_inv;

    J(0, 0) = fx_i * z_inv;
    J(0, 1) = 0.0;
    J(0, 2) = -fx_i * x * z_inv2;

    J(1, 0) = 0.0;
    J(1, 1) = fy_i * z_inv;
    J(1, 2) = -fy_i * y * z_inv2;
  }

  inline int width() const { return width_; }

  inline int height() const { return height_; }

  inline double scale() const { return scale_; }

  inline bool isInFrame(const Vector2i & obs, int boundary=0) const
  {
    if(obs[0]>=boundary && obs[0]<width()-boundary
        && obs[1]>=boundary && obs[1]<height()-boundary)
      return true;
    return false;
  }

  inline bool isInFrame(const Vector2i &obs, int boundary, int level) const
  {
    if(obs[0] >= boundary && obs[0] < width()/(1<<level)-boundary
        && obs[1] >= boundary && obs[1] <height()/(1<<level)-boundary)
      return true;
    return false;
  }
};

} // end namespace CSfM

#endif /* ABSTRACT_CAMERA_H_ */
