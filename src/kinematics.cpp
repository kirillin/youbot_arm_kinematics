#include "kinematics.h"

/*! The constructor for initialization kinematics by DH-parameters.*/
Kinematics::Kinematics(VectorNd _a, VectorNd _alpha, VectorNd _d, VectorNd _theta):
            a(_a), alpha(_alpha), d(_d), theta(_theta) {
}

/*! The method for solving Forward Kinematic Problem.
 *
 * \param[in] q The generalized coordinates of manipulator.
 * \param[in] from The base frame from where the desired transformation.
 * \param[in] to The goal frame where the desired transformation.
 *
 * \returns s The vector `[x, y, z, r, p, y]`.
 * */
Vector6d Kinematics::forward(VectorNd q, int from, int to) {
  Vector6d s;
  Transform<double, 3, Affine> H = Transform<double, 3, Affine>::Identity();
  Transform<double, 3, Affine> t;
  for(int i = from; i < to; i++) {
    t = Transform<double, 3, Affine>::Identity(); /*!< `t = Roz * Toz * Tox * Rox` */
    t.rotate(AngleAxisd(theta(i) - q(i), Vector3d::UnitZ()));
    t.translate(Vector3d(0, 0, d(i)));
    t.translate(Vector3d(a(i), 0, 0));
    t.rotate(AngleAxisd(alpha(i), Vector3d::UnitX()));
    H = H * t;
  }
  Vector3d rpy = H.rotation().eulerAngles(2,1,0).matrix().reverse();
  Vector3d tr = H.translation().matrix();
  s << tr, rpy;
  return s;
}

ConfigurationsOfManipulator Kinematics::inverse(Vector6d s) {
  ConfigurationsOfManipulator conf_manip;

  double px = s(0), py = s(1), pz = s(2);
  Matrix3d pr;
  pr = AngleAxisd(s(5), Vector3d::UnitZ())
      * AngleAxisd(s(4), Vector3d::UnitY())
      * AngleAxisd(s(3), Vector3d::UnitX());
  // theta_1
  double theta_1_I = atan2(py, px);
  double theta_1_II = atan2(-py, -px);

  //theta_2
  double s5 = pr(0, 0) * sin(theta_1_I) - pr(1, 0) * cos(theta_1_I);
  double c5 = pr(0, 1) * sin(theta_1_I) - pr(1, 1) * cos(theta_1_I);
  double theta_5_I = atan2(s5, c5);
  s5 = pr(0, 0) * sin(theta_1_II) - pr(1, 0) * cos(theta_1_II);
  c5 = pr(0, 1) * sin(theta_1_II) - pr(1, 1) * cos(theta_1_II);
  double theta_5_II = atan2(s5, c5);

  // theta_234
  double c234 = -pr(2, 2);
  double s234 = pr(0, 2) * cos(theta_1_I) + pr(1, 2) * sin(theta_1_I);
  double theta_234_I = atan2(s234, c234);
  s234 = pr(0, 2) * cos(theta_1_II) + pr(1, 2) * sin(theta_1_II);
  double theta_234_II = atan2(s234, c234);

  // theta_3
  Vector3d v = get_vector_1to4_frame(theta_1_I, pr, Vector3d(s(0), s(1), s(2)));
  double xr = v(0), yr = v(1), zr = v(2);
  double numerator_0 = xr * xr + yr * yr + zr * zr - a(1) * a(1) - a(2) * a(2);
  double denominator_0 = 2 * a(1) * a(2);
  double cosq3_I = numerator_0 / denominator_0;

  if (cosq3_I <= 1) {
    double theta_3_I = -atan2(sqrt(1.0 - cosq3_I * cosq3_I), cosq3_I);
    double theta_3_II = atan2(sqrt(1.0 - cosq3_I * cosq3_I), cosq3_I);

    double phi_I = atan2(yr, xr);
    double beta_I = atan2(a(2) * sin(abs(theta_3_I)), a(1) + a(2) * cos(abs(theta_3_I)));
    double beta_II = atan2(a(2) * sin(abs(theta_3_II)), a(1) + a(2) * cos(abs(theta_3_II)));

    double theta_2_I = phi_I + beta_I;
    double theta_2_II = phi_I - beta_II;

    double theta_4_I = theta_234_I - theta_2_I - theta_3_I;
    double theta_4_II = theta_234_I - theta_2_II - theta_3_II;

    conf_manip.qs.row(0) << theta_1_I, theta_2_I, theta_3_I, theta_4_I, theta_5_I,
    conf_manip.qs.row(1) << theta_1_I, theta_2_II, theta_3_II, theta_4_II, theta_5_I;
  } else {
    clog << "No solutions for q_I and q_II" << endl;
    conf_manip.solves.head(2) << false, false;
  }

  v = get_vector_1to4_frame(theta_1_II, pr, Vector3d(s(0), s(1), s(2)));
  xr = v(0); yr = v(1); zr = v(2);
  xr -= FLT_EPSILON;
  numerator_0 = xr * xr + yr * yr + zr * zr - a(1)*a(1) - a(2)*a(2);
  denominator_0 = 2 * a(1) * a(2);
  double cosq3_II = numerator_0 / denominator_0  - copysign(1.0, numerator_0) * FLT_EPSILON;

  if (cosq3_II <= 1) {
    double theta_3_III = -atan2(sqrt(1.0 - cosq3_II * cosq3_II), cosq3_II);
    double theta_3_IV = atan2(sqrt(1.0 - cosq3_II * cosq3_II), cosq3_II);

    double phi_II = atan2(yr, xr);
    double beta_III = atan2(a(2) * sin(abs(theta_3_III)), a(1) + a(2) * cos(abs(theta_3_III)));
    double beta_IV = atan2(a(2) * sin(abs(theta_3_IV)), a(1) + a(2) * cos(abs(theta_3_IV)));

    double theta_2_III = phi_II + beta_III;
    double theta_2_IV = phi_II - beta_IV;

    double theta_4_III = theta_234_II - theta_2_III - theta_3_III;
    double theta_4_IV = theta_234_II - theta_2_IV - theta_3_IV;

    conf_manip.qs.row(2) << theta_1_II, theta_2_III, theta_3_III, theta_4_III, theta_5_II;
    conf_manip.qs.row(3) << theta_1_II, theta_2_IV, theta_3_IV, theta_4_IV, theta_5_II;
  } else {
    clog << "No solutions for q_III and q_IV" << endl;
    conf_manip.solves.tail(2) << false, false;
  }

  for(int j = 0; j < N; j++) {
    for (int i = 0; i < 4; i++) {
      conf_manip.qs(i, j) = theta(j) - conf_manip.qs(i, j);
      conf_manip.qs(i, j) = normalized_angle(conf_manip.qs(i, j), j);
    }
  }
  return conf_manip;
}

Vector3d Kinematics::get_vector_1to4_frame(double theta, Matrix<double, 3, 3> pr, Vector3d p) {
  Vector3d v;
  double px = p(0), py = p(1), pz = p(2);
  double c1 = cos(theta), s1 = sin(theta);
  double xr = (py - d(4) * pr(1,2)) * s1 + (px - d(4) * pr(0,2)) * c1 - a(0);
  double yr = pz - d(0) - d(4) * pr(2, 2);
  double zr = 0.0;
  v << xr, yr, zr;
  return v;
}

double Kinematics::normalized_angle(double q, int j) {
  if (j != 2) {
    while (q < 0.0) {
      q += 2. * M_PI;
    }
    while (q >= 2 * M_PI) {
      q -= 2. * M_PI;
    }
  } else {
    while (q > 0.0) {
      q -= 2. * M_PI;
    }
    while (q <= -2 * M_PI) {
      q += 2. * M_PI;
    }
   }
  return q;
}


//
//int main() {
//  Vector5f v1; v1 << 0.033f, 0.155f, 0.135f, 0.0f, 0.0f;
//  Vector5f v2; v2 << M_PI/2.0f, 0.0f, 0.0f, M_PI/2.0f, 0.0f;
//  Vector5f v3; v3 << 0.147f, 0.0f, 0.0f, 0.0f, 0.218f;
//  Vector5f v4; v4 << M_PI*169.0/180.0, M_PI*65.0/180.0+M_PI/2, -M_PI*146.0/180.0, M_PI*102.5/180.0+M_PI/2, M_PI*167.5/180.0;
//  Kinematics ks(v1,v2,v3,v4);
//
//  // FK test
////  Vector5f q; q << 10.0, 2.0, 3.0, 4.0, 5.0;
////  Vector6f s = ks.forward(q, 0, 5);
//
//  // IK test
//  Vector6f s; s << 1,1,1,0.1,0.5,2;
//  MatrixXd qs = ks.inverse(s);
//
//  return 0;
//}