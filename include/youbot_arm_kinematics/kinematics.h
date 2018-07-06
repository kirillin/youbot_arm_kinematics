#ifndef KINEMATICS_
#define KINEMATICS_

#include <iostream>
#include <iomanip>
#include <limits>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

#include <youbot_arm_kinematics/IK.h> // NEVER confuse the title of the header files.
                                      // It can take two days of your life...
using namespace std;
using namespace Eigen;

const int N = 5; // quantity joints

typedef Matrix<double, 1, N> VectorNd;
typedef Matrix<double, 6, 1> Vector6d; // [x y z r p y]

/*! Contains achievable configurations of manipulator. */
struct ConfigurationsOfManipulator {
  /*!
   * #Example:
   *
   * ```cpp
   * qs = [
   *    [q1,q2,q3,q4,q5],
   *    [q1,q2,q3,q4,q5],
   *    [q1,q2,q3,q4,q5],
   *    [q1,q2,q3,q4,q5]
   * ];
   * solves = [
   *    true;
   *    true;
   *    false;
   *    false
   * ];
   * ```
   *
   */
  Matrix<double, 4, N> qs;    /*!< Matrix for four configurations of manipulator. */
  Matrix<double, 4, 1> solves;            /*!< achievable or not */
  ConfigurationsOfManipulator() {
    qs = Matrix<double, 4, N>::Zero();
    solves = Matrix<double, 4, 1>::Ones();
  }
};

/*! No comments -> no users of you code -> no problems */
class Kinematics {
  VectorNd a, alpha, d, theta;
 public:
  Kinematics() {};
  Kinematics(VectorNd _a, VectorNd _alpha, VectorNd _d, VectorNd _theta);

  Vector6d forward(VectorNd q, int from=0, int to=5);
  ConfigurationsOfManipulator inverse(Vector6d s);

  void get_jacobi(VectorNd q, Matrix<double, 6, N>& J);
  void get_pinv_jacobi_num(VectorNd q, Matrix<double, N, 6>& J);
  void get_pinv_jacobi(VectorNd q, Matrix<double, N, 6>& J);

  Vector3d get_vector_1to4_frame(double theta, Matrix<double, 3, 3> pr, Vector3d p);
  double normalized_angle(double q, int j);
  // bool check_on_limits(VectorNd q, Matrix5d limits);

  // void workspace_random_movements();
  // void movements_to_music(); /*! For example: different frequency for each joint*/
};

#endif
