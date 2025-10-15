#include <cmath>
#include <limits>

#include <vikit/omni_radtan_camera.h>

namespace vk {

namespace {
inline bool isAlmostZero(double v) {
  return std::abs(v) < 1e-12;
}
}

OmniRadtanCamera::OmniRadtanCamera(int width, int height, double scale,
                                   double xi, double fx, double fy, double cx, double cy,
                                   double k1, double k2, double p1, double p2, double k3)
    : AbstractCamera(static_cast<int>(width * scale), static_cast<int>(height * scale), scale),
      fx_(fx * scale),
      fy_(fy * scale),
      cx_(cx * scale),
      cy_(cy * scale),
      xi_(xi),
      k1_(k1),
      k2_(k2),
      p1_(p1),
      p2_(p2),
      k3_(k3) {
  has_distortion_ = !(isAlmostZero(k1_) && isAlmostZero(k2_) && isAlmostZero(p1_) &&
                      isAlmostZero(p2_) && isAlmostZero(k3_));
}

Eigen::Vector2d OmniRadtanCamera::undistortRadtan(const Eigen::Vector2d &distorted) const {
  if (!has_distortion_)
    return distorted;

  Eigen::Vector2d undistorted = distorted;
  for (int i = 0; i < 8; ++i) {
    const double x = undistorted.x();
    const double y = undistorted.y();
    const double r2 = x * x + y * y;
    const double radial = 1.0 + k1_ * r2 + k2_ * r2 * r2 + k3_ * r2 * r2 * r2;
    const double delta_x = 2.0 * p1_ * x * y + p2_ * (r2 + 2.0 * x * x);
    const double delta_y = p1_ * (r2 + 2.0 * y * y) + 2.0 * p2_ * x * y;
    const double inv_radial = (std::abs(radial) > 1e-12) ? 1.0 / radial : 1.0;

    const double new_x = (distorted.x() - delta_x) * inv_radial;
    const double new_y = (distorted.y() - delta_y) * inv_radial;

    const Eigen::Vector2d new_pt(new_x, new_y);
    if ((new_pt - undistorted).norm() < 1e-12)
      break;
    undistorted = new_pt;
  }
  return undistorted;
}

void OmniRadtanCamera::distortRadtan(const double x, const double y, double &xd, double &yd) const {
  if (!has_distortion_) {
    xd = x;
    yd = y;
    return;
  }

  const double r2 = x * x + y * y;
  const double r4 = r2 * r2;
  const double r6 = r4 * r2;
  const double radial = 1.0 + k1_ * r2 + k2_ * r4 + k3_ * r6;
  xd = x * radial + 2.0 * p1_ * x * y + p2_ * (r2 + 2.0 * x * x);
  yd = y * radial + p1_ * (r2 + 2.0 * y * y) + 2.0 * p2_ * x * y;
}

Eigen::Vector3d OmniRadtanCamera::cam2world(const double &u, const double &v) const {
  const double mx = (u - cx_) / fx_;
  const double my = (v - cy_) / fy_;
  const Eigen::Vector2d undistorted = undistortRadtan(Eigen::Vector2d(mx, my));

  const double x = undistorted.x();
  const double y = undistorted.y();
  const double r2 = x * x + y * y;
  const double sqrt_term = std::sqrt(1.0 + (1.0 - xi_ * xi_) * r2);
  const double factor = (xi_ + sqrt_term) / (r2 + 1.0);

  Eigen::Vector3d ray;
  ray << factor * x, factor * y, factor - xi_;
  return ray.normalized();
}

Eigen::Vector3d OmniRadtanCamera::cam2world(const Eigen::Vector2d &px) const {
  return cam2world(px.x(), px.y());
}

Eigen::Vector2d OmniRadtanCamera::world2cam(const Eigen::Vector3d &xyz_c) const {
  const double X = xyz_c.x();
  const double Y = xyz_c.y();
  const double Z = xyz_c.z();
  const double norm = xyz_c.norm();
  if (norm < 1e-12)
    return Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::quiet_NaN());
  const double denom = Z + xi_ * norm;

  if (denom <= 1e-12)
    return Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::quiet_NaN());

  const double x = X / denom;
  const double y = Y / denom;

  double xd = x;
  double yd = y;
  distortRadtan(x, y, xd, yd);

  Eigen::Vector2d px;
  px.x() = fx_ * xd + cx_;
  px.y() = fy_ * yd + cy_;
  return px;
}

Eigen::Vector2d OmniRadtanCamera::world2cam(const Eigen::Vector2d &uv) const {
  double xd = uv.x();
  double yd = uv.y();
  distortRadtan(uv.x(), uv.y(), xd, yd);

  Eigen::Vector2d px;
  px.x() = fx_ * xd + cx_;
  px.y() = fy_ * yd + cy_;
  return px;
}

void OmniRadtanCamera::projectionJacobian(const Eigen::Vector3d &xyz_c,
                                          Eigen::Matrix<double, 2, 3> &J) const {
  const double X = xyz_c.x();
  const double Y = xyz_c.y();
  const double Z = xyz_c.z();
  const double norm = xyz_c.norm();
  const double denom = Z + xi_ * norm;

  if (norm < 1e-12 || denom <= 1e-12) {
    J.setZero();
    return;
  }

  const double denom2 = denom * denom;
  const double inv_norm = 1.0 / norm;
  const double d_dX = xi_ * X * inv_norm;
  const double d_dY = xi_ * Y * inv_norm;
  const double d_dZ = 1.0 + xi_ * Z * inv_norm;

  const double x = X / denom;
  const double y = Y / denom;

  const double dx_dX = (denom - X * d_dX) / denom2;
  const double dx_dY = -(X * d_dY) / denom2;
  const double dx_dZ = -(X * d_dZ) / denom2;

  const double dy_dX = -(Y * d_dX) / denom2;
  const double dy_dY = (denom - Y * d_dY) / denom2;
  const double dy_dZ = -(Y * d_dZ) / denom2;

  double xd = x;
  double yd = y;
  distortRadtan(x, y, xd, yd);

  const double r2 = x * x + y * y;
  const double radial = 1.0 + k1_ * r2 + k2_ * r2 * r2 + k3_ * r2 * r2 * r2;
  const double radial_derivative = k1_ + 2.0 * k2_ * r2 + 3.0 * k3_ * r2 * r2;

  const double dxd_dx = radial + 2.0 * x * x * radial_derivative + 2.0 * p1_ * y +
                        6.0 * p2_ * x;
  const double dxd_dy = 2.0 * x * y * radial_derivative + 2.0 * p1_ * x + 2.0 * p2_ * y;
  const double dyd_dx = 2.0 * x * y * radial_derivative + 2.0 * p1_ * x + 2.0 * p2_ * y;
  const double dyd_dy = radial + 2.0 * y * y * radial_derivative + 6.0 * p1_ * y + 2.0 * p2_ * x;

  const double dpx_dX = fx_ * (dxd_dx * dx_dX + dxd_dy * dy_dX);
  const double dpx_dY = fx_ * (dxd_dx * dx_dY + dxd_dy * dy_dY);
  const double dpx_dZ = fx_ * (dxd_dx * dx_dZ + dxd_dy * dy_dZ);

  const double dpy_dX = fy_ * (dyd_dx * dx_dX + dyd_dy * dy_dX);
  const double dpy_dY = fy_ * (dyd_dx * dx_dY + dyd_dy * dy_dY);
  const double dpy_dZ = fy_ * (dyd_dx * dx_dZ + dyd_dy * dy_dZ);

  J(0, 0) = dpx_dX;
  J(0, 1) = dpx_dY;
  J(0, 2) = dpx_dZ;
  J(1, 0) = dpy_dX;
  J(1, 1) = dpy_dY;
  J(1, 2) = dpy_dZ;
}

} // namespace vk
