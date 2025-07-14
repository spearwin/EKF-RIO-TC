#include "rio_utils/strapdown.h"

#include <iostream>

namespace rio {
Strapdown::Strapdown(const double local_gravity)
    : local_gravity_(0, 0, local_gravity) {}

NavigationSolution Strapdown::propagate(const NavigationSolution nav_sol_prev,
                                        const Vector3 a_hat,
                                        const Vector3 w_hat, const Real dt) {
  const Matrix3 R_ItoG_prev = nav_sol_prev.getR_ItoG();
  const Vector3 v_IinG_prev = nav_sol_prev.v_IinG;

  // propagate attitude with forth order runge kutta
  const Vector4 zero_omega(0, w_hat.x(), w_hat.y(), w_hat.z());
  const Quaternion q_ItoG_prev = nav_sol_prev.getAttitude();
  const Vector4 q_ItoG_prev_vec(q_ItoG_prev.w(), q_ItoG_prev.x(),
                                q_ItoG_prev.y(), q_ItoG_prev.z());

  const Vector4 k_1(0.5 * getQLeftMatrix(q_ItoG_prev_vec) * zero_omega);
  const Vector4 k_2(0.5 * getQLeftMatrix(q_ItoG_prev_vec + k_1 * dt / 2) *
                    zero_omega);
  const Vector4 k_3(0.5 * getQLeftMatrix(q_ItoG_prev_vec + k_2 * dt / 2) *
                    zero_omega);
  const Vector4 k_4(0.5 * getQLeftMatrix(q_ItoG_prev_vec + k_3 * dt) *
                    zero_omega);

  const Vector4 new_q_vec =
      q_ItoG_prev_vec + (k_1 + 2 * k_2 + 2 * k_3 + k_4) * dt / 6;
  const Quaternion q_ItoG(new_q_vec[0], new_q_vec[1], new_q_vec[2],
                          new_q_vec[3]);

  // propagate velocity using Simpson's rule
  const Quaternion q_b1_b = q_ItoG_prev * q_ItoG.inverse();
  const Matrix3 C_b1_b = q_b1_b.toRotationMatrix();

  // clang-format off
  const Vector3 s_l = (a_hat
                       + 4 * (a_hat + 0.5 * (C_b1_b - Matrix3::Identity()) *
                       a_hat)
                       + (a_hat + (C_b1_b - Matrix3::Identity()) * a_hat)) *
                      dt / 6;
  // clang-format on

  Vector3 v_IinG =
      nav_sol_prev.v_IinG + R_ItoG_prev * s_l + local_gravity_ * dt;

  // propagate position using chained Simpson's rule
  // clang-format off
  const Vector3 v_01 = (a_hat
                        + 4 * (a_hat + 0.25 * (C_b1_b - Matrix3::Identity())
                        * a_hat)
                        + (a_hat + 0.5 * (C_b1_b - Matrix3::Identity()) *
                        a_hat))* dt / 12;
  // clang-format on

  const Vector3 y_l = (4 * v_01 + s_l) * dt / 6;
  const Vector3 p_n1_n =
      v_IinG_prev * dt + R_ItoG_prev * y_l + 0.5 * local_gravity_ * dt * dt;
  const Vector3 p_IinG = Vector3(nav_sol_prev.getPosition() + p_n1_n);

  return NavigationSolution(p_IinG, q_ItoG, v_IinG);
}

Matrix4 Strapdown::getQLeftMatrix(const Vector4 &v) {
  Matrix4 m;
  // clang-format off
  m << v[0],  -v[1],  -v[2],  -v[3],
       v[1],   v[0],  -v[3],   v[2],
       v[2],   v[3],   v[0],  -v[1],
       v[3],  -v[2],   v[1],   v[0];
  // clang-format on

  return m;
}

}  // namespace rio
