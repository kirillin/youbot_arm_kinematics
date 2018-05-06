#include "kinematics.h"

class FKtest {
 private:
  ros::NodeHandle _nh;
  ros::Subscriber js;
  Kinematics ks;

 public:
  FKtest() {}
  FKtest(Kinematics _ks): ks(_ks) {
    js = _nh.subscribe("joint_states", 1, &FKtest::callback, this);
  }

  void callback(const sensor_msgs::JointState::ConstPtr& js) {
    VectorNd q;
    for (int i = 0; i < 5; i++) {
      q(i) = js->position[i];
    }

    /*! Compute and test Jacobi matrices*/
    Matrix<double, 6, N> J; /* just Jacobi matrix*/
    Matrix<double, N, 6> piJ;  /* pseudo inversed Jacobi matrix, that above */
    Matrix<double, N, 6> piJ2;  /* pseudo inversed Jacobi matrix, analytic */
    ks.get_jacobi(q, J);
    ks.get_pinv_jacobi_num(q, piJ);
    ks.get_pinv_jacobi(q, piJ2);

    cout << "\nJ =\n" << J << endl;
    cout << "piJ_num ="
         << (piJ * piJ.transpose()).determinant()
         << endl << piJ << endl;
    cout << "piJ_alg =\n"
         << (piJ2 * piJ2.transpose()).determinant()
         << endl << piJ2 << endl << endl;

    Vector6d s = ks.forward(q, 0, 5);
    cout << "s: " << s.transpose() << endl; // [x y z r p y]

    static tf::TransformBroadcaster br;
    tf::Quaternion qt;
    qt.setRPY(s(3),s(4),s(5));
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(s(0),s(1),s(2)));
    transform.setRotation(qt);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ground_link", "tool_link"));
  }

  void spin() {
    ros::Rate R(100);
    while(_nh.ok()) {
      ros::spinOnce();
      R.sleep();
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "fk_test_node");

  VectorNd v1; v1 << 0.033, 0.155, 0.135, 0.0, 0.0;
  VectorNd v2; v2 << M_PI/2.0, 0.0, 0.0, M_PI/2.0, 0.0;
  VectorNd v3; v3 << 0.147, 0.0, 0.0, 0.0, 0.218;
  VectorNd v4; v4 << M_PI*169.0/180.0, M_PI*65.0/180.0+M_PI/2, -M_PI*146.0/180.0, M_PI*102.5/180.0+M_PI/2, M_PI*167.5/180.0;
  Kinematics ks(v1, v2, v3, v4);

  FKtest fk(ks);
  fk.spin();
  return 0;
}